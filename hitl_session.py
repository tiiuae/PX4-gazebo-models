#!/usr/bin/env python3
"""Launch Gazebo HITL, spawn the model, and clean up the owned process group."""
import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

CREATE_SERVICE = '/world/default/create'


def model_udp_ports(model):
    """Fixed UDP bind ports used by directly configured HITL bridges."""
    ports = []
    for plugin in ET.parse(model).getroot().iter('plugin'):
        if Path(plugin.get('filename', '')).name != 'libmavlink_hitl_gazebosim.so':
            continue
        if plugin.findtext('use_tcp', '0').strip().lower() in ('1', 'true'):
            continue
        # Match the plugin defaults; port zero requests an ephemeral OS assignment.
        ports.append(int(plugin.findtext('mavlink_udp_local_port', '0')))
        secondary = plugin.findtext('secondary_mavlink_addr')
        secondary_enabled = secondary is not None and secondary.strip() != 'INADDR_ANY'
        if secondary_enabled or plugin.find('secondary_mavlink_udp_local_port') is not None:
            ports.append(int(plugin.findtext('secondary_mavlink_udp_local_port', '1')))
    if any(port < 0 or port > 65535 for port in ports):
        raise RuntimeError('Invalid HITL UDP port in ' + str(model))
    return [port for port in ports if port]


def check_udp_ports(ports):
    # Hold all probes together to detect duplicate ports within the model too.
    with ExitStack() as stack:
        for port in ports:
            probe = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            try:
                probe.bind(('0.0.0.0', port))
            except OSError as error:
                raise RuntimeError(f'HITL UDP port {port} is unavailable: {error}. '
                                   'Another simulator may already be running. Inspect with '
                                   f"ss -lunp 'sport = :{port}', then stop the existing session first; "
                                   'GZ_PARTITION does not isolate UDP ports.') from error


def owned_udp_ports(root_pid):
    """Inspect socket ownership in the launched process tree (Linux /proc)."""
    pending = [root_pid]
    visited = set()
    inodes = set()
    while pending:
        pid = pending.pop()
        if pid in visited:
            continue
        visited.add(pid)
        proc = Path('/proc') / str(pid)
        for children in proc.glob('task/*/children'):
            try:
                pending.extend(int(value) for value in children.read_text().split())
            except (FileNotFoundError, ProcessLookupError):
                pass
        try:
            descriptors = list((proc / 'fd').iterdir())
        except (FileNotFoundError, ProcessLookupError):
            continue
        for descriptor in descriptors:
            try:
                target = os.readlink(descriptor)
            except (FileNotFoundError, ProcessLookupError):
                continue
            if target.startswith('socket:['):
                inodes.add(target[8:-1])
    ports = set()
    for line in Path('/proc/net/udp').read_text().splitlines()[1:]:
        fields = line.split()
        if len(fields) > 9 and fields[9] in inodes:
            ports.add(int(fields[1].rsplit(':', 1)[1], 16))
    return ports


def wait_model_sockets(process, ports, timeout=15):
    if not ports:
        return
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError('Gazebo exited during HITL plugin initialization (status ' +
                               str(process.returncode) + ')')
        if set(ports).issubset(owned_udp_ports(process.pid)):
            return
        time.sleep(0.1)
    raise RuntimeError('Timed out waiting for the launched simulator to own HITL UDP ports: ' +
                       ', '.join(map(str, ports)))


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
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--name', default='HITL_Drone')
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=30)
    parser.add_argument('options', nargs=argparse.REMAINDER)
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
        options = args.options[1:] if args.options[:1] == ['--'] else args.options
        validate_options(options)
        ports = model_udp_ports(args.model)
        check_udp_ports(ports)
        assert_unused(env)
        print('GZ_PARTITION=' + env['GZ_PARTITION'], flush=True)
        process = subprocess.Popen(['gz', 'sim', str(args.world), '-r', *options],
                                   env=env, start_new_session=True)
        wait_ready(process, env, args.timeout)
        create_model(args.model.resolve(), args.name, env)
        if process.poll() is not None:
            raise RuntimeError('Gazebo exited during model creation')
        # A true create reply only queues model creation; plugin loading happens later.
        wait_model_sockets(process, ports)
        print('HITL model created: ' + args.name, flush=True)
        return process.wait()
    except KeyboardInterrupt:
        return 130
    except (RuntimeError, OSError, subprocess.SubprocessError, ET.ParseError, ValueError) as error:
        print('HITL: ' + str(error), file=sys.stderr)
        return 1
    finally:
        if process is not None:
            terminate(process)


if __name__ == '__main__':
    sys.exit(main())
