"""Fly the Gymnasium environment against a live simulator and record evidence.

Runs a seeded, bounded random policy for a few episodes, checks that the
Gymnasium contract holds (space membership, five-tuple steps, seeded goal
reproducibility), and writes a JSON result. Simulation only.
"""

import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'PythonClient'))

import numpy as np
import gymnasium as gym

from airsim_gym import AirSimQuadrotorEnv


def json_safe(value):
    """Keep NaN/Inf visible as strings instead of letting json.dumps raise."""
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=41451)
    parser.add_argument('--vehicle', default='SimpleFlight')
    parser.add_argument('--episodes', type=int, default=2)
    parser.add_argument('--steps', type=int, default=25)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--step-duration', type=float, default=0.2)
    parser.add_argument('--camera', default=None,
                        help='camera name; when set the observation is a Dict of state and RGB image')
    parser.add_argument('--image-height', type=int, default=36)
    parser.add_argument('--image-width', type=int, default=64)
    parser.add_argument('--reset-mode', choices=('teleport', 'land'), default='teleport',
                        help="'land' puts the vehicle down before each reset; needed for PX4")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'checks': [], 'episodes': [], 'seed': args.seed,
              'scope': 'Gymnasium contract and bounded random policy; not a learning result'}
    env = None

    def save():
        (args.output / 'result.json').write_text(
            json.dumps(json_safe(report), indent=2, allow_nan=False) + '\n')

    def check(name, value, detail=None):
        report['checks'].append({'name': name, 'passed': bool(value), 'detail': detail})
        save()
        print(('PASS ' if value else 'FAIL ') + name, flush=True)
        if not value:
            raise AssertionError(name)

    try:
        env = AirSimQuadrotorEnv(host=args.host, port=args.port, vehicle_name=args.vehicle,
                                 step_duration=args.step_duration, max_episode_steps=args.steps,
                                 rpc_timeout=40.0, reset_mode=args.reset_mode,
                                 camera_name=args.camera,
                                 image_shape=(args.image_height, args.image_width))
        check('Gymnasium env contract', isinstance(env, gym.Env))
        observation, info = env.reset(seed=args.seed)
        check('reset returns two values', isinstance(info, dict) and 'target_enu' in info)
        detail = ({k: list(np.asarray(v).shape) for k, v in observation.items()}
                  if isinstance(observation, dict) else observation.tolist())
        check('observation in space', env.observation_space.contains(observation), detail)
        first_offset = np.asarray(info['target_offset_enu'])
        check('goal within target radius', float(np.linalg.norm(first_offset[:2])) <= env.target_radius + 1e-6,
              first_offset.tolist())
        observation, info = env.reset(seed=args.seed)
        # Hover drift between resets shifts absolute coordinates slightly; the
        # seeded quantity is the goal offset from the current hover pose.
        np.testing.assert_allclose(np.asarray(info['target_offset_enu']), first_offset, atol=1e-9)
        check('same seed reproduces goal offset', True, info['target_offset_enu'])

        for episode in range(args.episodes):
            observation, info = env.reset(seed=args.seed + episode)
            record = {'episode': episode, 'target_enu': info['target_enu'],
                      'start_distance_m': info['distance_m'], 'steps': [],
                      'total_reward': 0.0, 'terminated': False, 'truncated': False}
            generator = np.random.default_rng(args.seed + episode)
            for step_index in range(args.steps):
                action = generator.uniform(env.action_space.low, env.action_space.high).astype(np.float32)
                observation, reward, terminated, truncated, info = env.step(action)
                check('step %d observation in space' % step_index,
                      env.observation_space.contains(observation))
                check('reward finite', np.isfinite(reward))
                record['total_reward'] += float(reward)
                record['steps'].append({'reward': float(reward), 'distance_m': info['distance_m'],
                                        'timestamp': info['timestamp'],
                                        'landed_state': info['landed_state']})
                if terminated or truncated:
                    record['terminated'], record['truncated'] = bool(terminated), bool(truncated)
                    break
            record['final_distance_m'] = record['steps'][-1]['distance_m']
            record['step_count'] = len(record['steps'])
            stamps = [s['timestamp'] for s in record['steps']]
            check('simulation clock advances', all(a < b for a, b in zip(stamps, stamps[1:])), stamps[:3])
            report['episodes'].append(record)
            save()
        report['status'] = 'passed'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = repr(error)
        print(repr(error), flush=True)
    finally:
        if env is not None:
            env.close()
        save()
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())
