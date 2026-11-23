"""Train a PPO policy on AirSimQuadrotorEnv with Stable-Baselines3.

Bounded, simulation-only training run used to prove the learning loop works
end to end: environment -> SB3 -> policy rollout -> checkpoint. It is not a
benchmark and does not claim the policy solves the task at small step counts.

``result.json`` is rewritten after every finished episode (not only at the
end), ``episodes.jsonl`` gets one line per episode, and checkpoints are kept,
so an interrupted or failed run still leaves inspectable evidence.
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
from gymnasium import spaces
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

from airsim_gym import AirSimQuadrotorEnv


def json_safe(value):
    """Convert numpy scalars and non-finite floats into valid JSON values.

    ``allow_nan=False`` is used everywhere so a corrupt number cannot pass as
    evidence; NaN/Inf become the strings 'nan'/'inf' instead of raising, which
    keeps the rest of the record writable when the simulator misbehaves.
    """
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def summarize_episodes(path):
    if not path.exists():
        return {'count': 0}
    episodes = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()
                if line.strip()]
    returns = [episode.get('return') for episode in episodes
               if isinstance(episode.get('return'), (int, float))]
    return {
        'count': len(episodes),
        'returns': returns,
        'mean_return': float(np.mean(returns)) if returns else None,
        'successes': sum(1 for episode in episodes if episode.get('success')),
        'collisions': sum(1 for episode in episodes if episode.get('collision')),
        'out_of_bounds': sum(1 for episode in episodes if episode.get('out_of_bounds')),
        'final_distances_m': [episode.get('distance_m') for episode in episodes],
    }


class EpisodeRecorder(BaseCallback):
    """Append one JSONL line per finished episode and refresh result.json."""

    def __init__(self, path, on_episode=None):
        super().__init__()
        self.path = path
        self.on_episode = on_episode
        self._previous_wall_s = None

    def _on_step(self):
        infos = self.locals.get('infos')
        dones = self.locals.get('dones')
        if dones is None or infos is None:
            return True
        for done, info in zip(dones, infos):
            if not done:
                continue
            record = {'num_timesteps': int(self.num_timesteps)}
            episode = info.get('episode')
            if episode is not None:
                # Monitor's 't' is wall time since the Monitor was created,
                # not the duration of this episode; keep it under a name that
                # says so and derive the per-episode duration separately.
                wall = float(episode['t'])
                record['return'] = float(episode['r'])
                record['length'] = int(episode['l'])
                record['wall_since_monitor_start_s'] = wall
                if self._previous_wall_s is not None:
                    record['wall_episode_s'] = round(wall - self._previous_wall_s, 3)
                self._previous_wall_s = wall
            for key in ('success', 'collision', 'out_of_bounds', 'distance_m',
                        'target_offset_enu'):
                if key in info:
                    record[key] = info[key]
            with self.path.open('a', encoding='utf-8') as handle:
                handle.write(json.dumps(json_safe(record), allow_nan=False) + '\n')
            if self.on_episode is not None:
                self.on_episode()
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=41451)
    parser.add_argument('--vehicle', default='SimpleFlight')
    parser.add_argument('--total-timesteps', type=int, default=4096)
    parser.add_argument('--seed', type=int, default=3)
    parser.add_argument('--step-duration', type=float, default=0.2)
    parser.add_argument('--max-episode-steps', type=int, default=100)
    parser.add_argument('--n-steps', type=int, default=512)
    parser.add_argument('--batch-size', type=int, default=128)
    parser.add_argument('--learning-rate', type=float, default=3e-4)
    parser.add_argument('--save-freq', type=int, default=2048,
                        help='checkpoint every N steps; 0 disables checkpoints')
    parser.add_argument('--camera', default=None,
                        help='camera name; when set the observation is a Dict of state and RGB image')
    parser.add_argument('--image-height', type=int, default=36)
    parser.add_argument('--image-width', type=int, default=64)
    parser.add_argument('--reset-mode', choices=('teleport', 'land'), default='teleport',
                        help="'land' puts the vehicle down before each reset; needed for PX4")
    parser.add_argument('--skip-check-env', action='store_true')
    args = parser.parse_args()
    try:
        args.output.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        print('output directory already exists: %s' % args.output, flush=True)
        return 2
    episodes_path = args.output / 'episodes.jsonl'
    report = {'status': 'running', 'scope': 'learning-loop integration run; not a benchmark',
              'config': {k: v for k, v in vars(args).items() if k != 'output'},
              'versions': {'stable_baselines3': stable_baselines3.__version__,
                           'torch': torch.__version__,
                           'numpy': np.__version__}}
    env = None
    model = None
    started = time.time()
    exit_code = 0

    def save():
        (args.output / 'result.json').write_text(
            json.dumps(json_safe(report), indent=2, allow_nan=False) + '\n', encoding='utf-8')

    def refresh_episode_summary():
        report['episodes'] = summarize_episodes(episodes_path)
        save()

    try:
        env = AirSimQuadrotorEnv(host=args.host, port=args.port, vehicle_name=args.vehicle,
                                 step_duration=args.step_duration,
                                 max_episode_steps=args.max_episode_steps,
                                 rpc_timeout=40.0, reset_mode=args.reset_mode,
                                 camera_name=args.camera,
                                 image_shape=(args.image_height, args.image_width))
        if not args.skip_check_env:
            # SB3's own contract check; any warning that is an error stops here.
            check_env(env, warn=True, skip_render_check=True)
            report['sb3_check_env'] = 'passed'
            save()

        vec = DummyVecEnv([lambda: Monitor(env)])
        vec.seed(args.seed)
        callbacks = [EpisodeRecorder(episodes_path, on_episode=refresh_episode_summary)]
        if args.save_freq > 0:
            callbacks.append(CheckpointCallback(
                save_freq=args.save_freq, save_path=str(args.output / 'checkpoints'),
                name_prefix='ppo'))

        # Dict observations carry an image; SB3 needs MultiInputPolicy and a
        # small CNN to process it. The state-only environment keeps the small
        # MLP it has always used.
        if isinstance(env.observation_space, spaces.Dict):
            # CombinedExtractor runs NatureCNN on the image and concatenates
            # the state features; cnn_output_dim is its own keyword (it has no
            # features_dim argument).
            policy = 'MultiInputPolicy'
            policy_kwargs = dict(
                features_extractor_kwargs=dict(cnn_output_dim=64),
                net_arch=[64, 64])
        else:
            policy = 'MlpPolicy'
            policy_kwargs = dict(net_arch=[64, 64])
        model = PPO(policy, vec, seed=args.seed, device='cpu', verbose=1,
                    n_steps=args.n_steps, batch_size=args.batch_size,
                    learning_rate=args.learning_rate,
                    policy_kwargs=policy_kwargs)
        # Keep the untrained policy so evaluation can compare before/after on
        # the same seeds through the same code path.
        try:
            model.save(args.output / 'model-init')
        except Exception as error:
            report['init_model_save_error'] = repr(error)
        model.learn(total_timesteps=args.total_timesteps, callback=callbacks)
        model.save(args.output / 'model')
        report['status'] = 'passed'
    except KeyboardInterrupt:
        report['status'] = 'interrupted'
        exit_code = 130
        if model is not None:
            try:
                model.save(args.output / 'model-interrupted')
            except Exception as error:
                report['interrupt_save_error'] = repr(error)
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = repr(error)
        print(repr(error), flush=True)
        exit_code = 1
    finally:
        if model is not None and report['status'] not in ('passed', 'interrupted'):
            # Keep whatever was learned so a failed run still leaves evidence.
            try:
                model.save(args.output / 'model-partial')
            except Exception as error:
                report['partial_save_error'] = repr(error)
        if model is not None:
            # SB3 creates the logger inside learn(); a failure before that
            # must not mask the original error here. Dump() clears
            # name_to_value, so this records only values logged after the
            # last dump, not a full history.
            logger = getattr(model, '_logger', None)
            if logger is not None:
                report['logger_last_values'] = {
                    key: float(value) for key, value in logger.name_to_value.items()
                    if isinstance(value, (int, float, np.integer, np.floating))}
        if env is not None:
            try:
                env.close()
            except Exception as error:
                report['close_error'] = repr(error)
        report['elapsed_s'] = round(time.time() - started, 1)
        report['episodes'] = summarize_episodes(episodes_path)
        save()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
