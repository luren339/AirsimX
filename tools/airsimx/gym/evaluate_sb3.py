"""Evaluate a trained policy on AirSimQuadrotorEnv and record evidence.

Runs a fixed, seeded set of episodes with a deterministic policy (unless
``--stochastic`` is given) and writes success rate, returns and final
distances to JSON. Simulation only.

The report records the full environment configuration and the policy and
SB3 versions next to the model path, so a result can be matched to the exact
setup that produced it. ``result.json`` is rewritten after every episode.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'PythonClient'))

import numpy as np
import stable_baselines3
import torch
from stable_baselines3 import PPO

from airsim_gym import AirSimQuadrotorEnv


def json_safe(value):
    """Convert numpy scalars and non-finite floats into valid JSON values."""
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
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=41451)
    parser.add_argument('--vehicle', default='SimpleFlight')
    parser.add_argument('--episodes', type=int, default=10)
    parser.add_argument('--seed-base', type=int, default=1000)
    parser.add_argument('--step-duration', type=float, default=0.2)
    parser.add_argument('--max-episode-steps', type=int, default=100)
    parser.add_argument('--camera', default=None,
                        help='camera name; when set the observation is a Dict of state and RGB image')
    parser.add_argument('--image-height', type=int, default=36)
    parser.add_argument('--image-width', type=int, default=64)
    parser.add_argument('--reset-mode', choices=('teleport', 'land'), default='teleport',
                        help="'land' puts the vehicle down before each reset; needed for PX4")
    parser.add_argument('--stochastic', action='store_true')
    args = parser.parse_args()
    try:
        args.output.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        print('output directory already exists: %s' % args.output, flush=True)
        return 2
    report = {'status': 'running', 'scope': 'policy evaluation; not a benchmark',
              'model_path': str(args.model), 'episodes': []}
    env = None
    exit_code = 0
    started = time.time()

    def save():
        (args.output / 'result.json').write_text(
            json.dumps(json_safe(report), indent=2, allow_nan=False) + '\n', encoding='utf-8')

    try:
        model = PPO.load(args.model, device='cpu')
        report['model'] = {'algorithm': type(model).__name__,
                           'policy': type(model.policy).__name__,
                           'device': str(model.device)}
        report['versions'] = {'stable_baselines3': stable_baselines3.__version__,
                              'torch': torch.__version__,
                              'numpy': np.__version__}
        env = AirSimQuadrotorEnv(host=args.host, port=args.port, vehicle_name=args.vehicle,
                                 step_duration=args.step_duration,
                                 max_episode_steps=args.max_episode_steps,
                                 rpc_timeout=40.0, reset_mode=args.reset_mode,
                                 camera_name=args.camera,
                                 image_shape=(args.image_height, args.image_width))
        report['env_config'] = {key: env.__dict__[key] for key in
                                ('host', 'port', 'vehicle', 'step_duration',
                                 'max_episode_steps', 'hover_height', 'target_radius',
                                 'success_radius', 'world_radius', 'max_velocity_enu',
                                 'max_yaw_rate_deg')}
        save()

        for index in range(args.episodes):
            seed = args.seed_base + index
            observation, info = env.reset(seed=seed)
            record = {'episode': index, 'seed': seed,
                      'target_offset_enu': info['target_offset_enu'],
                      'start_distance_m': info['distance_m'],
                      'total_reward': 0.0, 'steps': 0,
                      'success': False, 'collision': False, 'out_of_bounds': False}
            while True:
                action, _ = model.predict(observation, deterministic=not args.stochastic)
                observation, reward, terminated, truncated, info = env.step(action)
                record['total_reward'] += float(reward)
                record['steps'] += 1
                if terminated or truncated:
                    record.update(success=bool(info['success']),
                                  collision=bool(info['collision']),
                                  out_of_bounds=bool(info['out_of_bounds']),
                                  truncated=bool(truncated))
                    break
            record['final_distance_m'] = float(info['distance_m'])
            report['episodes'].append(record)
            save()
            print('episode %d: return %.2f, final distance %.2f m, success %s'
                  % (index, record['total_reward'], record['final_distance_m'],
                     record['success']), flush=True)

        returns = [episode['total_reward'] for episode in report['episodes']]
        distances = [episode['final_distance_m'] for episode in report['episodes']]
        report['summary'] = {
            'episodes': len(returns),
            'success_rate': sum(1 for episode in report['episodes'] if episode['success'])
                            / max(1, len(returns)),
            'collisions': sum(1 for episode in report['episodes'] if episode['collision']),
            'mean_return': float(np.mean(returns)) if returns else 0.0,
            'mean_final_distance_m': float(np.mean(distances)) if distances else 0.0,
            'deterministic': not args.stochastic,
        }
        report['status'] = 'passed'
    except KeyboardInterrupt:
        report['status'] = 'interrupted'
        exit_code = 130
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = repr(error)
        print(repr(error), flush=True)
        exit_code = 1
    finally:
        if env is not None:
            try:
                env.close()
            except Exception as error:
                report['close_error'] = repr(error)
        report['elapsed_s'] = round(time.time() - started, 1)
        save()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
