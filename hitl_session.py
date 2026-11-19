#!/usr/bin/env python3
"""Bounded Gazebo Harmonic discovery/spawning shared by HITL entry points."""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import uuid

CREATE_SERVICE = '/world/default/create'


def select_partition(env):
    """Call once per session, then copy this environment to every participant."""
    env['GZ_PARTITION'] = env.get('GZ_PARTITION') or 'px4-hitl-' + uuid.uuid4().hex
    return env['GZ_PARTITION']


def services(env):
    result = subprocess.run(['gz', 'service', '-l'], env=env, capture_output=True,
                            text=True, timeout=3, check=True)
    return result.stdout.splitlines()


def assert_unused(env):
    if CREATE_SERVICE in services(env):
        raise RuntimeError('default world already advertised in GZ_PARTITION=' +
                           env['GZ_PARTITION'] + '; stop it or use a distinct partition')


def wait_ready(process, env, timeout=30):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError('Gazebo exited before default world was ready (status ' +
                               str(process.returncode) + ')')
        try:
            if CREATE_SERVICE in services(env):
                if process.poll() is not None:
                    raise RuntimeError('Gazebo exited during discovery')
                return
        except subprocess.TimeoutExpired:
            pass
        time.sleep(0.2)
    raise RuntimeError('Timed out waiting for ' + CREATE_SERVICE +
                       '; check Gazebo logs, UserCommands plugin and partition')


def create_model(model, name, env):
    # JSON string escaping is also valid for protobuf text string fields.
    request = ('sdf_filename: ' + json.dumps(str(model)) + ', name: ' + json.dumps(name) +
               ' pose: {position: {x: 1.01, y: 0.98, z: 0.83}}')
    result = subprocess.run(['gz', 'service', '-s', CREATE_SERVICE,
                             '--reqtype', 'gz.msgs.EntityFactory',
                             '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
                             '--req', request], env=env, capture_output=True,
                            text=True, timeout=10)
    print(result.stdout, end='', flush=True)
    if result.returncode or not re.search(r'\bdata:\s*true\b', result.stdout):
        raise RuntimeError('Model creation failed: ' + result.stdout + result.stderr)


def terminate(process):
    """Reap only the process group created by this invocation, including GUI children."""
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        process.poll()
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.05)
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


def validate_options(options):
    # Only options that cannot select a different world/server/configuration are forwarded.
    allowed = {'-s', '--server-only', '--headless-rendering', '-v', '--verbose'}
    value_options = {'-v', '--verbose', '--render-engine', '--render-engine-server'}
    index = 0
    while index < len(options):
        option = options[index]
        if option in value_options:
            index += 1
            if index == len(options) or options[index].startswith('-'):
                raise RuntimeError('Missing value for ' + option)
        elif option not in allowed and not any(
                option.startswith(key + '=') for key in value_options if key.startswith('--')):
            raise RuntimeError('Unsupported Gazebo option ' + option +
                               '; launcher owns the default world. See SALUKI_FT_HITL.md')
        index += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    for command in ('launch', 'create'):
        child = sub.add_parser(command)
        child.add_argument('--model', type=Path, required=True)
        child.add_argument('--name', default='HITL_Drone')
        if command == 'launch':
            child.add_argument('--world', type=Path, required=True)
            child.add_argument('--timeout', type=float, default=30)
            child.add_argument('options', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    env = os.environ.copy()
    select_partition(env)
    process = None

    def interrupted(signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    try:
        if not args.model.is_file():
            raise RuntimeError('Model not found: ' + str(args.model))
        if args.command == 'create':
            create_model(args.model, args.name, env)
            return 0
        options = args.options[1:] if args.options[:1] == ['--'] else args.options
        validate_options(options)
        assert_unused(env)
        print('GZ_PARTITION=' + env['GZ_PARTITION'], flush=True)
        process = subprocess.Popen(['gz', 'sim', str(args.world), '-r', *options],
                                   env=env, start_new_session=True)
        wait_ready(process, env, args.timeout)
        create_model(args.model.resolve(), args.name, env)
        if process.poll() is not None:
            raise RuntimeError('Gazebo exited during model creation')
        print('HITL model created: ' + args.name, flush=True)
        return process.wait()
    except KeyboardInterrupt:
        return 130
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        print('HITL: ' + str(error), file=sys.stderr)
        return 1
    finally:
        if process is not None:
            terminate(process)


if __name__ == '__main__':
    sys.exit(main())
