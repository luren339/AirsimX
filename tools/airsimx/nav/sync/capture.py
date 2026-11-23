"""Record an existing ROS Jazzy simulation graph. Never starts a pilot or publishes commands."""
import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
import uuid

import yaml

from evidence import DIAG, NODE, TOPICS, dump, sha

HERE = Path(__file__).resolve().parent


def now():
    return datetime.now(timezone.utc).isoformat()


def launch(argv, log, **kwargs):
    with log.open('x', encoding='utf-8') as f:
        return subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=f, stderr=subprocess.STDOUT,
                                start_new_session=True, **kwargs)


def finish(p):
    """Only signal children started in a new process group by this invocation."""
    if p is None or p.poll() is not None:
        return False
    for sig, grace in ((signal.SIGINT, 12), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            return False
        try:
            p.wait(timeout=grace)
            return sig != signal.SIGINT
        except subprocess.TimeoutExpired:
            pass
    raise RuntimeError(f'child {p.pid} did not exit')


def endpoint(info):
    q = info.qos_profile
    return {'node': info.node_namespace.rstrip('/') + '/' + info.node_name,
            'type': info.topic_type, 'gid': bytes(info.endpoint_gid).hex(),
            'reliability': q.reliability.name, 'durability': q.durability.name,
            'history': q.history.name, 'depth': q.depth}


def record(a):
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    out = a.out.resolve()
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    manifest = {'format': 1, 'scene': a.scene, 'state': 'incomplete', 'started_utc': now(),
                'wall_limit_s': a.wall_seconds, 'domain': os.getenv('ROS_DOMAIN_ID', '0'),
                'ros_distro': os.getenv('ROS_DISTRO'), 'planner_node': NODE, 'image_id': a.image_id,
                'inputs': {}, 'installed_planner': {}, 'topics': TOPICS,
                'time_basis': 'MCAP receipt = latest ROS /clock, not wall time or sensor acquisition time'}
    mf = out / 'manifest.json'
    dump(mf, manifest)
    rec = param = node = None
    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda number, frame: stop.append(signal.Signals(number).name))
    try:
        if a.scene == 'blocks' and not a.settings:
            raise ValueError('Blocks requires --settings (the AirSim settings used for this run)')
        for src, name in ((a.source_config, 'source-params.yaml'), (a.settings, 'airsim-settings.json'),
                          (HERE / 'qos.yaml', 'qos.yaml')):
            if src:
                shutil.copyfile(src, out / name)
                manifest['inputs'][name] = {'source': str(src), 'sha256': sha(out / name)}
        if set(yaml.safe_load((out / 'qos.yaml').read_text())) != set(TOPICS):
            raise ValueError('topic list and QoS profile differ')
        exe = a.workspace / 'install/drone_city_nav/lib/drone_city_nav/production_mppi_node'
        manifest['installed_planner'] = {'path': str(exe), 'sha256': sha(exe)}
        git = subprocess.run(['git', '--no-optional-locks', '-C', str(a.workspace), 'rev-parse', 'HEAD'],
                             capture_output=True, text=True, timeout=4, check=True)
        manifest['navigation_commit'] = git.stdout.strip()
        manifest['tools_sha256'] = {f: sha(HERE / f) for f in ('capture.py', 'verify.py', 'evidence.py', 'qos.yaml')}
        name = 'airsimx_sync_' + uuid.uuid4().hex[:12]
        cmd = ['ros2', 'bag', 'record', '--storage', 'mcap', '--output', str(out / 'bag'), '--use-sim-time',
               '--disable-keyboard-controls', '--node-name', name, '--qos-profile-overrides-path', str(out / 'qos.yaml'),
               '--topics', *TOPICS]
        manifest['record_argv'] = cmd
        dump(mf, manifest)
        env = dict(os.environ, ROS_LOG_DIR=str(out / 'roslog'), RCUTILS_COLORIZED_OUTPUT='0')
        rec = launch(cmd, out / 'rosbag.log', env=env)
        deadline = time.monotonic() + a.wall_seconds
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node(name + '_observer', enable_rosout=False, start_parameter_services=False)
        endpoints = {t: {} for t in TOPICS}
        dump(out / 'publishers.json', {t: [] for t in TOPICS})
        attempt, par_started, next_probe = 0, 0., 0.
        print(f'Recording into {out}; Ctrl-C stops the recorder only. Wall limit {a.wall_seconds} s.', flush=True)
        while not stop and time.monotonic() < deadline:
            if rec.poll() is not None:
                raise RuntimeError(f'rosbag exited unexpectedly ({rec.returncode}); see rosbag.log')
            mono = time.monotonic()
            if mono >= next_probe:
                added = False
                for topic in TOPICS:
                    for info in node.get_publishers_info_by_topic(topic):
                        e = endpoint(info)
                        if e['gid'] not in endpoints[topic]:
                            endpoints[topic][e['gid']] = e
                            added = True
                if added:
                    dump(out / 'publishers.json', {t: list(v.values()) for t, v in endpoints.items()})
                next_probe = mono + 0.5
            if param is not None:
                if mono - par_started > 8 and param.poll() is None:
                    finish(param)
                if param.poll() is not None:
                    if param.returncode == 0:
                        text = (out / f'param-{attempt}.log').read_text(encoding='utf-8')
                        # A successful CLI exit without the requested node is not a snapshot.
                        document = yaml.safe_load(text)
                        values = (document or {}).get(NODE, {}).get('ros__parameters', {})
                        if 'mission_goal_sequence_xyz_m' in values and 'use_sim_time' in values:
                            dest = out / 'effective-planner.yaml'
                            dest.write_text(text, encoding='utf-8')
                            manifest['effective_planner'] = {'sha256': sha(dest), 'read_utc': now(), 'node': NODE}
                            dump(mf, manifest)
                    param = None
            if (param is None and 'effective_planner' not in manifest and mono - par_started >= 2 and
                    any(e['node'] == NODE for e in endpoints[DIAG].values())):
                attempt += 1
                par_started = mono
                param = launch(['ros2', 'param', 'dump', '--no-daemon', '--timeout', '3', NODE],
                               out / f'param-{attempt}.log', env=env)
            time.sleep(0.1)
        manifest['stop_reason'] = stop[0] if stop else 'wall_limit'
    except Exception as err:
        manifest['error'] = f'{type(err).__name__}: {err}'
    finally:
        forced = finish(rec)
        finish(param)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        manifest['recorder_exit'] = None if rec is None else rec.returncode
        manifest['ended_utc'] = now()
        if (out / 'publishers.json').is_file():
            manifest['publishers_sha256'] = sha(out / 'publishers.json')
        bag = out / 'bag'
        files = [p for p in sorted(bag.glob('*')) if p.is_file() and (p.suffix == '.mcap' or p.name == 'metadata.yaml')]
        manifest['bag_sha256'] = {p.name: sha(p) for p in files}
        if (not forced and not manifest.get('error') and manifest['recorder_exit'] == 0 and
                (bag / 'metadata.yaml').is_file() and any(p.suffix == '.mcap' for p in files)):
            manifest['state'] = 'closed'  # Only verify.py can certify a search witness.
        dump(mf, manifest)
    print(f"{manifest['state']}: {mf}" + (f" ({manifest['error']})" if 'error' in manifest else ''), flush=True)
    return 0 if manifest['state'] == 'closed' else 2


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, required=True, help='new directory; existing runs are never overwritten')
    p.add_argument('--source-config', type=Path, required=True, help='navigation input YAML (not effective parameters)')
    p.add_argument('--settings', type=Path, help='AirSim settings supplied to UE; required for Blocks')
    p.add_argument('--workspace', type=Path, default=Path('/workspace'), help='read-only navigation install/source root')
    p.add_argument('--scene', choices=('blocks', 'frozen-rehearsal'), default='blocks')
    p.add_argument('--image-id', required=True, help='container image ID from docker inspect (sha256:...)')
    p.add_argument('--wall-seconds', type=int, default=180, help='wall-time limit even if /clock stops (1..3600)')
    a = p.parse_args()
    if not 1 <= a.wall_seconds <= 3600 or not re.fullmatch(r'sha256:[0-9a-f]{64}', a.image_id):
        p.error('invalid wall limit or container image ID')
    return record(a)


if __name__ == '__main__':
    sys.exit(main())
