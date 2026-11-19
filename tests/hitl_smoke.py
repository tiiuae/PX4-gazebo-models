#!/usr/bin/env python3
"""Host-only single/FT loopback startup and transport isolation checks. No controller setup."""
import argparse
from contextlib import ExitStack
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET

GZ_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(GZ_DIR))
import hitl_session as session


def environment(workspace):
    env = os.environ.copy()
    build = workspace / 'build/px4_sitl_default'
    for key, paths in {
            'GZ_SIM_RESOURCE_PATH': [GZ_DIR / 'models', GZ_DIR / 'worlds'],
            'GZ_SIM_SYSTEM_PLUGIN_PATH': [build / 'build_gz-sim_plugins',
                                         build / 'src/modules/simulation/gz_plugins']}.items():
        env[key] = ':'.join([env[key]] if env.get(key) else [])
        env[key] = ':'.join(filter(None, [env[key], *(str(p) for p in paths)]))
    env['GZ_SIM_SERVER_CONFIG_PATH'] = str(workspace / 'src/modules/simulation/gz_bridge/server.config')
    # Always override inherited partitions: smoke checks must never join a hardware session.
    env['GZ_PARTITION'] = 'px4-smoke-' + uuid.uuid4().hex
    return env


def scene(env):
    result = subprocess.run(['gz', 'service', '-s', '/world/default/scene/info',
                             '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene',
                             '--timeout', '2000', '--req', ''], env=env,
                            capture_output=True, text=True, timeout=5, check=True)
    if not result.stdout.strip():
        raise RuntimeError('No scene response in ' + env['GZ_PARTITION'])
    return result.stdout


def camera_frames(env, logs, variant):
    topics = subprocess.run(['gz', 'topic', '-l'], env=env, capture_output=True,
                            text=True, timeout=5, check=True).stdout.splitlines()
    images = [topic for topic in topics if topic.endswith('/image') and
              ('third-person-camera' in topic or 'onboard-gimbal-camera' in topic)]
    if len(images) != 2:
        raise RuntimeError('Expected both X500 camera image topics, found: ' + str(images))
    for index, topic in enumerate(images):
        result = subprocess.run(['gz', 'topic', '-e', '-n', '1', '-t', topic], env=env,
                                capture_output=True, text=True, timeout=15, check=True)
        if not all(re.search(field + r':\s*[1-9][0-9]*', result.stdout)
                   for field in ['width', 'height']) or 'data:' not in result.stdout:
            raise RuntimeError('Invalid camera frame on ' + topic)
        (logs / (variant + f'-camera-{index}.txt')).write_text(
            'topic=' + topic + '\n' + re.sub(r'data:.*', 'data: <omitted>', result.stdout))
    print('PASS camera frames: ' + ', '.join(images), flush=True)


def stop_launcher(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            session.terminate(process)
            raise RuntimeError('Launcher did not clean up within 10 seconds')


def scenario(variant, channels, args, logs):
    workspace = GZ_DIR.parents[2]
    env = environment(workspace)
    other_env = environment(workspace)
    print(f'{variant}: GZ_PARTITION={env["GZ_PARTITION"]}; comparison={other_env["GZ_PARTITION"]}', flush=True)
    with ExitStack() as stack:
        model_dir = Path(stack.enter_context(tempfile.TemporaryDirectory(prefix='px4-hitl-')))
        # Reserve distinct local UDP ports; hold the receive socket throughout the test.
        sockets = [stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
                   for _ in range(channels + 1)]
        for sock in sockets:
            sock.bind(('127.0.0.1', 0))
        local_ports = [sock.getsockname()[1] for sock in sockets[:-1]]
        receiver = sockets[-1]
        receiver.settimeout(0.2)
        remote_port = receiver.getsockname()[1]
        tree = ET.parse(GZ_DIR / 'models/ssrc_holybro_x500' / (variant + '.sdf'))
        plugin = tree.getroot().find('./model/plugin')
        replacements = {'mavlink_addr': '127.0.0.1', 'mavlink_udp_remote_port': str(remote_port),
                        'mavlink_udp_local_port': str(local_ports[0])}
        if channels == 2:
            replacements.update(secondary_mavlink_addr='127.0.0.1',
                                secondary_mavlink_udp_local_port=str(local_ports[1]))
        for tag, value in replacements.items():
            node = plugin.find(tag)
            if node is None:
                raise RuntimeError('Missing SDF field: ' + tag)
            node.text = value
        model = model_dir / (variant + '.sdf')
        tree.write(model, encoding='utf-8', xml_declaration=True)
        (logs / (variant + '-ports.txt')).write_text(
            f'partition={env["GZ_PARTITION"]}\nlocal={local_ports}\nremote={remote_port}\n')
        comparison_log = stack.enter_context((logs / (variant + '-comparison.log')).open('w'))
        other = subprocess.Popen(['gz', 'sim', '-s', '--headless-rendering', '-r',
                                  str(GZ_DIR / 'worlds/default.sdf')], env=other_env,
                                 stdout=comparison_log, stderr=subprocess.STDOUT, start_new_session=True)
        stack.callback(session.terminate, other)
        session.wait_ready(other, other_env, args.timeout)
        for sock in sockets[:-1]:
            sock.close()
        log_path = logs / (variant + '.log')
        if args.automated:
            sys.path.insert(0, str(workspace / 'test/mavsdk_tests'))
            from integration_test_runner import process_helper as ph
            server = ph.GzHarmonicServer(str(workspace), str(logs), 'ssrc_holybro_x500',
                                          'smoke', 1, False, 'build/px4_sitl_default')
            server.env.update(env)
            server.set_log_filename(str(log_path))
            stack.callback(server.stop)
            server.start()
            if not server.has_started_ok():
                raise RuntimeError('Automated server readiness failed')
            spawner = ph.GzHarmonicModelSpawnRunner(str(workspace), str(logs),
                                                   'ssrc_holybro_x500', 'smoke', False,
                                                   'build/px4_sitl_default', str(model.with_suffix('')))
            spawner.env.update(env)
            spawner.set_log_filename(str(logs / (variant + '-spawn.log')))
            stack.callback(spawner.stop)
            spawner.start()
            if not spawner.has_started_ok():
                raise RuntimeError('Automated model creation failed')
            process = server.process
            model_name = 'ssrc_holybro_x500'
            cleanup = server.stop
        else:
            log = stack.enter_context(log_path.open('w'))
            process = subprocess.Popen([str(GZ_DIR / 'hitl_run.sh'), str(model), '--headless', '-v', '3'],
                                       env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            cleanup = lambda: stop_launcher(process)
            stack.callback(cleanup)
            model_name = 'HITL_Drone'
        deadline = time.monotonic() + args.timeout
        seen = set()
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(f'{variant}: launcher exited with {process.returncode}; see {log_path}')
            output = log_path.read_text(errors='replace')
            if re.search(r'Failed to load (?:system )?plugin|Could not find shared library|bind failed', output, re.I):
                raise RuntimeError(f'{variant}: required plugin/socket failure; see {log_path}')
            try:
                payload, source = receiver.recvfrom(65535)
                if payload and source[1] in local_ports:
                    seen.add(source[1])
            except socket.timeout:
                pass
            if (args.automated or 'HITL model created:' in output) and seen == set(local_ports):
                break
        else:
            raise RuntimeError(f'{variant}: timeout; channels received {seen}, expected {local_ports}; see {log_path}')
        if args.camera:
            camera_frames(env, logs, variant)
        # Service replies alone do not establish discovery isolation.
        if f'name: "{model_name}"' not in scene(env):
            raise RuntimeError('Smoke model absent from its partition')
        if f'name: "{model_name}"' in scene(other_env):
            raise RuntimeError('Smoke model leaked into comparison partition')
        cleanup()
        if other.poll() is not None or f'name: "{model_name}"' in scene(other_env):
            raise RuntimeError('Smoke cleanup affected comparison simulator')
        print(f'PASS {variant}: creation, {channels} UDP channel(s), plugin loading and partition isolation', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir', type=Path, default=Path('logs/hitl-smoke-' + time.strftime('%Y%m%d-%H%M%S')))
    parser.add_argument('--camera', action='store_true', help='Also require an image from each X500 camera')
    parser.add_argument('--automated', action='store_true', help='Use MAVSDK runner server/spawn classes; never prepare controllers')
    parser.add_argument('--timeout', type=float, default=60)
    args = parser.parse_args()
    args.log_dir.mkdir(parents=True, exist_ok=False)
    print('Logs: ' + str(args.log_dir.resolve()), flush=True)
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        for variant, channels in [('model_hitl', 1), ('model_ft_hitl', 2)]:
            scenario(variant, channels, args, args.log_dir)
    except (RuntimeError, OSError, subprocess.SubprocessError, KeyboardInterrupt) as error:
        print('FAIL: ' + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
