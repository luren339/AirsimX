import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np

try:
    from airsim_gym import AirSimQuadrotorEnv
except ImportError as error:  # gymnasium is the optional 'gym' extra
    raise unittest.SkipTest('airsim_gym needs the gym extra: %s' % error)


class _Vector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x_val, self.y_val, self.z_val = float(x), float(y), float(z)


class _Kinematics:
    def __init__(self):
        self.position = _Vector(0.0, 0.0, -3.0)  # NED, airborne at 3 m
        self.linear_velocity = _Vector()
        self.angular_velocity = _Vector()
        self.orientation = _Vector(0.0, 0.0, 0.0)
        self.orientation.w_val = 1.0


class _State:
    def __init__(self, kinematics, timestamp, landed_state):
        self.kinematics_estimated = kinematics
        self.timestamp = timestamp
        self.landed_state = landed_state


class _Collision:
    def __init__(self, has_collided=False, time_stamp=0):
        self.has_collided = has_collided
        self.time_stamp = time_stamp


class _ImageResponse:
    def __init__(self, width, height, data):
        self.width = width
        self.height = height
        self.image_data_uint8 = data


class _Done:
    def __init__(self, value=True):
        self._value = value

    def get(self, timeout=None):
        return self._value


LANDED = 0  # airsim.LandedState.Landed
FLYING = 1


class FakeClient:
    """Minimal simulator stand-in that integrates velocity commands."""

    def __init__(self, position_ned=(0.0, 0.0, -3.0), landed_state=LANDED):
        self.initial_ned = position_ned
        self.kinematics = _Kinematics()
        self.kinematics.position = _Vector(*position_ned)
        self.timestamp = 0
        self.landed_state = landed_state
        self.calls = []
        self.collision = _Collision()
        self.closed = False
        self.api_control = False
        self.armed = False

    def getMultirotorState(self, vehicle_name=''):
        return _State(self.kinematics, self.timestamp, self.landed_state)

    def simGetGroundTruthKinematics(self, vehicle_name=''):
        # The environment verifies climbs against ground truth; in the fake
        # the estimator and the truth are the same state.
        return self.kinematics

    def reset(self):
        self.calls.append(('reset',))
        self.kinematics = _Kinematics()
        self.kinematics.position = _Vector(*self.initial_ned)
        self.timestamp = 0
        self.landed_state = LANDED
        self.api_control = False
        self.armed = False

    def enableApiControl(self, enabled, vehicle_name=''):
        self.calls.append(('enableApiControl', bool(enabled)))
        self.api_control = bool(enabled)

    def armDisarm(self, arm, vehicle_name=''):
        self.calls.append(('armDisarm', bool(arm)))
        self.armed = bool(arm)
        return True

    def takeoffAsync(self, timeout_sec=20, vehicle_name=''):
        self.calls.append(('takeoffAsync',))
        self.kinematics.position.z_val = -3.0
        self.landed_state = FLYING
        self.timestamp += int(20e9)
        return _Done(True)

    def hoverAsync(self, vehicle_name=''):
        self.calls.append(('hoverAsync',))
        return _Done(True)

    def moveToPositionAsync(self, x, y, z, velocity, timeout_sec=30, yaw_mode=None, vehicle_name=''):
        self.calls.append(('moveToPositionAsync', x, y, z, velocity))
        # The real call resolves once the target is reached, so move all the
        # way in the call; the elapsed time follows the commanded speed.
        dx = x - self.kinematics.position.x_val
        dy = y - self.kinematics.position.y_val
        dz = z - self.kinematics.position.z_val
        duration = max(abs(dx), abs(dy), abs(dz)) / max(velocity, 0.1)
        self.kinematics.position = _Vector(x, y, z)
        self.landed_state = FLYING
        self.timestamp += max(1, int(duration * 1e9))
        return _Done(True)

    def moveByVelocityAsync(self, vx, vy, vz, duration, yaw_mode=None, vehicle_name=''):
        self.calls.append(('moveByVelocityAsync', vx, vy, vz, duration,
                           None if yaw_mode is None else (yaw_mode.is_rate, yaw_mode.yaw_or_rate)))
        self.kinematics.position.x_val += vx * duration
        self.kinematics.position.y_val += vy * duration
        self.kinematics.position.z_val += vz * duration
        self.kinematics.linear_velocity = _Vector(vx, vy, vz)
        self.timestamp += int(duration * 1e9)
        return _Done(True)

    def simGetCollisionInfo(self, vehicle_name=''):
        return self.collision

    def simGetImages(self, requests, vehicle_name='', external=False):
        self.calls.append(('simGetImages', tuple((r.camera_name, r.compress) for r in requests)))
        # AirSim returns raw B, G, R bytes when compress=False. Build a frame
        # whose top-left pixel is (B=10, G=20, R=30) and the rest (1, 2, 3).
        width, height = 8, 4
        frame = np.tile(np.array([1, 2, 3], dtype=np.uint8), (height, width, 1))
        frame[0, 0] = (10, 20, 30)
        return [_ImageResponse(width, height, frame.reshape(-1).tobytes())]

    def landAsync(self, timeout_sec=30, vehicle_name=''):
        self.calls.append(('landAsync',))
        self.kinematics.position.z_val = 0.0
        self.landed_state = LANDED
        return _Done(True)

    def close(self):
        self.closed = True


def make_env(client=None, **overrides):
    options = dict(step_duration=0.5, max_episode_steps=10, hover_height=3.0,
                   target_radius=8.0, success_radius=0.5, world_radius=50.0,
                   max_velocity_enu=2.0, max_yaw_rate_deg=30.0)
    options.update(overrides)
    return AirSimQuadrotorEnv(client=client or FakeClient(), **options)


class SpaceTests(unittest.TestCase):
    def test_spaces_match_documented_layout(self):
        env = make_env()
        self.assertEqual(env.observation_space.shape, (env.OBSERVATION_DIMENSION,))
        self.assertEqual(env.observation_space.shape, (13,))
        self.assertEqual(env.action_space.shape, (4,))
        np.testing.assert_allclose(env.action_space.low, [-2.0, -2.0, -2.0, -30.0])
        np.testing.assert_allclose(env.action_space.high, [2.0, 2.0, 2.0, 30.0])
        env.close()

    def test_invalid_geometry_rejected(self):
        with self.assertRaises(ValueError):
            make_env(success_radius=9.0)  # above target_radius
        with self.assertRaises(ValueError):
            make_env(step_duration=0.0)
        with self.assertRaises(ValueError):
            make_env(min_target_radius=0.4)  # below success_radius
        with self.assertRaises(ValueError):
            make_env(reset_mode='warp')


class ImageObservationTests(unittest.TestCase):
    def test_no_camera_keeps_plain_state_observation(self):
        env = make_env()
        observation, _ = env.reset(seed=0)
        self.assertIsInstance(observation, np.ndarray)
        self.assertEqual(observation.shape, (13,))
        env.close()

    def test_image_observation_is_dict_and_in_space(self):
        env = make_env(camera_name='0', image_shape=(4, 8))
        observation, _ = env.reset(seed=0)
        self.assertIsInstance(observation, dict)
        self.assertEqual(set(observation), {'state', 'image'})
        self.assertEqual(observation['state'].shape, (13,))
        self.assertEqual(observation['image'].shape, (4, 8, 3))
        self.assertEqual(observation['image'].dtype, np.uint8)
        self.assertTrue(env.observation_space.contains(observation))
        env.close()

    def test_image_is_converted_from_bgr_to_rgb(self):
        client = FakeClient()
        env = make_env(client, camera_name='0', image_shape=(4, 8))
        observation, _ = env.reset(seed=0)
        # AirSim wrote (B, G, R) = (10, 20, 30); the observation is RGB.
        np.testing.assert_array_equal(observation['image'][0, 0], [30, 20, 10])
        command = [c for c in client.calls if c[0] == 'simGetImages'][-1]
        self.assertEqual(command[1], (('0', False),))  # raw bytes, no PNG
        env.close()

    def test_image_is_resized_to_requested_shape(self):
        env = make_env(camera_name='0', image_shape=(2, 4))
        observation, _ = env.reset(seed=0)
        self.assertEqual(observation['image'].shape, (2, 4, 3))
        env.close()

    def test_invalid_image_shape_rejected(self):
        with self.assertRaises(ValueError):
            make_env(camera_name='0', image_shape=(0, 8))
        with self.assertRaises(ValueError):
            make_env(camera_name='0', image_shape=(8,))

    def test_step_returns_image_in_space(self):
        env = make_env(camera_name='0', image_shape=(4, 8),
                       target_radius=8.0, success_radius=0.5, world_radius=50.0)
        env.reset(seed=0, options={'target_enu': [50.0, 50.0, 3.0]})
        observation, reward, terminated, truncated, info = env.step(
            np.array([0.2, 0.0, 0.0, 0.0], dtype=np.float32))
        self.assertTrue(env.observation_space.contains(observation))
        env.close()


class ResetTests(unittest.TestCase):
    def test_observation_uses_enu(self):
        env = make_env(FakeClient(position_ned=(1.0, 2.0, -3.0)))
        observation, info = env.reset(seed=1)
        # NED north=1, east=2, down=-3 becomes ENU east=2, north=1, up=3.
        np.testing.assert_allclose(observation[:3], [2.0, 1.0, 3.0], atol=1e-6)
        self.assertTrue(env.observation_space.contains(observation))
        self.assertIn('target_enu', info)
        env.close()

    def test_reset_arms_and_takes_off(self):
        client = FakeClient()
        env = make_env(client)
        env.reset(seed=0)
        names = [call[0] for call in client.calls]
        self.assertIn('reset', names)
        self.assertIn('enableApiControl', names)
        self.assertIn('armDisarm', names)
        self.assertIn('takeoffAsync', names)
        self.assertTrue(client.armed)
        env.close()

    def test_seeded_targets_are_reproducible(self):
        env_a = make_env()
        env_b = make_env()
        _, info_a = env_a.reset(seed=1234)
        _, info_b = env_b.reset(seed=1234)
        self.assertEqual(info_a['target_offset_enu'], info_b['target_offset_enu'])
        _, info_c = env_b.reset(seed=9999)
        self.assertNotEqual(info_a['target_offset_enu'], info_c['target_offset_enu'])
        env_a.close()
        env_b.close()

    def test_target_stays_within_radius(self):
        env = make_env()
        for seed in range(20):
            _, info = env.reset(seed=seed)
            target = np.asarray(info['target_enu'])
            start = np.asarray(env._observe(env.client.getMultirotorState())[:3], dtype=float)
            distance = np.linalg.norm(target[:2] - start[:2])
            self.assertLessEqual(distance, env.target_radius + 1e-6)
            # A goal inside success_radius of the hover pose would already be
            # reached before the first action.
            self.assertGreaterEqual(distance, env.min_target_radius - 1e-6)
            self.assertAlmostEqual(target[2], start[2], places=6)
        env.close()

    def test_land_mode_lands_without_teleport(self):
        client = FakeClient()
        env = make_env(client, reset_mode='land')
        env.reset(seed=0)
        env.step(np.zeros(4, dtype=np.float32))
        client.calls.clear()
        env.reset(seed=1)
        names = [call[0] for call in client.calls]
        # A flying vehicle is put down, and the simulator's teleport-reset is
        # not used because firmware-backed vehicles cannot be teleported.
        self.assertIn('landAsync', names)
        self.assertNotIn('reset', names)
        env.close()

    def test_teleport_mode_does_not_land_first(self):
        client = FakeClient()
        env = make_env(client)
        env.reset(seed=0)
        env.step(np.zeros(4, dtype=np.float32))
        client.calls.clear()
        env.reset(seed=1)
        names = [call[0] for call in client.calls]
        self.assertNotIn('landAsync', names)
        env.close()

    def test_arm_failure_retries_then_raises(self):
        client = FakeClient()
        env = make_env(client, arm_timeout=0.3)
        calls = {'count': 0}
        original = client.armDisarm

        def refusing(arm, vehicle_name=''):
            calls['count'] += 1
            if arm:
                return False
            return original(arm, vehicle_name)

        client.armDisarm = refusing
        with self.assertRaises(RuntimeError):
            env.reset(seed=0)
        self.assertGreater(calls['count'], 1)  # retried, not a single attempt

    def test_seeded_targets_never_start_inside_success_radius(self):
        env = make_env()
        for seed in range(200):
            _, info = env.reset(seed=seed)
            self.assertGreater(np.linalg.norm(info['target_offset_enu'][:2]),
                               env.success_radius)
        env.close()


class StepTests(unittest.TestCase):
    def test_enu_action_maps_to_ned_command(self):
        client = FakeClient()
        env = make_env(client)
        env.reset(seed=0, options={'target_enu': [50.0, 50.0, 3.0]})
        env.step(np.array([1.0, 0.0, 0.0, 30.0], dtype=np.float32))
        command = [call for call in client.calls if call[0] == 'moveByVelocityAsync'][-1]
        # ENU east 1 m/s becomes NED east (vy=1); yaw rate flips sign.
        self.assertEqual(command[1:4], (0.0, 1.0, 0.0))
        self.assertEqual(command[5], (True, -30.0))
        env.close()

    def test_action_is_clipped(self):
        client = FakeClient()
        env = make_env(client)
        env.reset(seed=0, options={'target_enu': [50.0, 50.0, 3.0]})
        env.step(np.array([100.0, -100.0, 100.0, 500.0], dtype=np.float32))
        command = [call for call in client.calls if call[0] == 'moveByVelocityAsync'][-1]
        self.assertLessEqual(abs(command[1]), 2.0)
        self.assertLessEqual(abs(command[3]), 2.0)
        env.close()

    def test_wrong_action_shape_rejected(self):
        env = make_env()
        env.reset(seed=0)
        with self.assertRaises(ValueError):
            env.step(np.zeros(3, dtype=np.float32))
        env.close()

    def test_reward_rewards_progress(self):
        client = FakeClient()
        env = make_env(client)
        env.reset(seed=0, options={'target_enu': [10.0, 0.0, 3.0]})
        observation, reward, terminated, truncated, info = env.step(np.array([0.2, 0.0, 0.0, 0.0], dtype=np.float32))
        self.assertGreater(reward, 0.0)
        self.assertFalse(terminated)
        self.assertFalse(truncated)
        self.assertLess(info['distance_m'], 10.0)
        env.close()

    def test_success_terminates_with_bonus(self):
        client = FakeClient()
        env = make_env(client)
        env.reset(seed=0, options={'target_enu': [0.3, 0.0, 3.0]})
        observation, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        self.assertTrue(terminated)
        self.assertTrue(info['success'])
        self.assertGreaterEqual(reward, 10.0)
        env.close()

    def test_collision_terminates_with_penalty(self):
        client = FakeClient()
        env = make_env(client)
        env.reset(seed=0, options={'target_enu': [50.0, 0.0, 3.0]})
        client.collision = _Collision(has_collided=True, time_stamp=10)
        observation, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        self.assertTrue(terminated)
        self.assertTrue(info['collision'])
        self.assertLess(reward, 0.0)
        env.close()

    def test_ground_contact_while_landed_is_not_a_collision(self):
        client = FakeClient()
        env = make_env(client)
        env.reset(seed=0, options={'target_enu': [50.0, 0.0, 3.0]})
        client.landed_state = LANDED  # firmware resting on the ground
        client.collision = _Collision(has_collided=True, time_stamp=10)
        observation, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        self.assertFalse(info['collision'])
        self.assertFalse(terminated)
        env.close()

    def test_collision_that_lands_the_vehicle_still_counts(self):
        client = FakeClient()
        env = make_env(client)
        env.reset(seed=0, options={'target_enu': [50.0, 0.0, 3.0]})
        env.step(np.zeros(4, dtype=np.float32))  # airborne before the impact
        client.landed_state = LANDED  # the impact knocks it down in this step
        client.collision = _Collision(has_collided=True, time_stamp=10)
        observation, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        self.assertTrue(info['collision'])
        self.assertTrue(terminated)
        env.close()

    def test_close_lands_when_reset_fails_mid_climb(self):
        client = FakeClient()
        env = make_env(client, reset_mode='land')
        original = client.moveToPositionAsync

        def failing(*args, **kwargs):
            raise RuntimeError('climb aborted')

        client.moveToPositionAsync = failing
        with self.assertRaises(RuntimeError):
            env.reset(seed=0)
        client.moveToPositionAsync = original
        client.calls.clear()
        env.close()
        names = [call[0] for call in client.calls]
        # The climb had started, so the vehicle may be airborne: close() must
        # still land it rather than just releasing control.
        self.assertIn('landAsync', names)
        self.assertFalse(client.closed)  # injected client stays open

    def test_old_collision_before_reset_is_ignored(self):
        client = FakeClient()
        env = make_env(client)
        client.collision = _Collision(has_collided=True, time_stamp=500)
        env.reset(seed=0, options={'target_enu': [50.0, 0.0, 3.0]})
        client.collision = _Collision(has_collided=False, time_stamp=500)
        observation, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        self.assertFalse(info['collision'])
        env.close()

    def test_out_of_bounds_terminates(self):
        client = FakeClient()
        env = make_env(client, world_radius=5.0, target_radius=4.0)
        env.reset(seed=0, options={'target_enu': [0.0, 0.0, 3.0]})
        client.kinematics.position.y_val = 40.0  # NED east -> ENU east
        observation, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        self.assertTrue(terminated)
        self.assertTrue(info['out_of_bounds'])
        # The terminal observation must still satisfy the declared Box even
        # though the true position is past world_radius.
        self.assertTrue(env.observation_space.contains(observation))
        self.assertGreater(abs(info['position_ned'][1]), 5.0)
        env.close()

    def test_truncation_after_max_steps(self):
        client = FakeClient()
        env = make_env(client, max_episode_steps=2, target_radius=40.0)
        env.reset(seed=0)
        for _ in range(2):
            observation, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        self.assertTrue(truncated)
        self.assertEqual(info['steps'], 2)
        env.close()


class CloseTests(unittest.TestCase):
    def test_close_lands_and_releases(self):
        client = FakeClient()
        env = make_env(client)
        env.reset(seed=0)
        env.close()
        names = [call[0] for call in client.calls]
        self.assertIn('landAsync', names)
        self.assertIn('enableApiControl', names)
        self.assertEqual(client.calls[-1][1], False)
        self.assertFalse(client.armed)
        self.assertFalse(client.closed)  # injected clients stay open

    def test_owned_client_is_closed(self):
        client = FakeClient()
        env = make_env(client)
        env._owns_client = True
        env.reset(seed=0)
        env.close()
        self.assertTrue(client.closed)


if __name__ == '__main__':
    unittest.main()
