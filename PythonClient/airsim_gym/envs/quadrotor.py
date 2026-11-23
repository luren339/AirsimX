"""A Gymnasium environment that flies an AirSim multirotor to sampled goals.

Design notes:

- The observation and action spaces are expressed in ROS-style ENU/FLU
  coordinates, converted through ``airsim.frames``. The simulator keeps
  its native NED/FRD state, which is also reported in ``info`` so a policy
  log can be compared with flight-controller logs.
- Actions are velocity commands in ENU plus a yaw-rate channel, sent with
  ``moveByVelocityAsync`` and held for ``step_duration`` of *simulation*
  time. Waiting on the command future, rather than the wall clock, keeps
  episode semantics stable on a slow renderer or a remote simulator.
- The target is sampled with the environment's own ``np_random``, which
  Gymnasium seeds via ``reset(seed=...)``; the same seed yields the same
  goal sequence.
- Ending an episode never releases control in the air: the vehicle is told
  to hold, and only ``close()`` lands, disarms and releases control.
- ``client`` may be injected to run against a pre-configured connection or
  a fake; the unit tests use this.
- With ``camera_name`` set, observations become a ``Dict`` of the state
  vector and an RGB camera frame (``image_shape``, default 36x64). Without it
  the observation is the plain 13-value vector, as before.
"""

import math

import gymnasium as gym
from gymnasium import spaces
import numpy as np

import airsim
from airsim import frames

try:
    import cv2
except ImportError:  # pragma: no cover - opencv is a declared dependency
    cv2 = None


class AirSimQuadrotorEnv(gym.Env):
    """Fly a multirotor to randomly sampled goals using velocity commands."""

    metadata = {'render_modes': [], 'render_fps': 10}

    OBSERVATION_DIMENSION = 13

    def __init__(self,
                 host='127.0.0.1',
                 port=41451,
                 vehicle_name='SimpleFlight',
                 step_duration=0.2,
                 max_episode_steps=500,
                 hover_height=3.0,
                 target_radius=8.0,
                 success_radius=0.5,
                 min_target_radius=None,
                 world_radius=60.0,
                 max_velocity_enu=3.0,
                 max_yaw_rate_deg=45.0,
                 rpc_timeout=30.0,
                 reset_mode='teleport',
                 arm_timeout=90.0,
                 camera_name=None,
                 image_type=airsim.ImageType.Scene,
                 image_shape=(36, 64),
                 client=None):
        super().__init__()
        if step_duration <= 0:
            raise ValueError('step_duration must be positive')
        if hover_height <= 0 or success_radius <= 0:
            raise ValueError('hover_height and success_radius must be positive')
        if target_radius <= success_radius or world_radius <= target_radius:
            raise ValueError('expected success_radius < target_radius < world_radius')
        if min_target_radius is None:
            # Goals closer than this would already be inside success_radius at
            # the hover pose, so the episode could succeed without moving.
            min_target_radius = 2.0 * success_radius
        if not success_radius < min_target_radius < target_radius:
            raise ValueError('expected success_radius < min_target_radius < target_radius')
        if reset_mode not in ('teleport', 'land'):
            raise ValueError("reset_mode must be 'teleport' or 'land'")
        if camera_name is not None:
            if len(image_shape) != 2 or image_shape[0] <= 0 or image_shape[1] <= 0:
                raise ValueError('image_shape must be (height, width) with positive values')
        self.host = host
        self.port = port
        self.vehicle = vehicle_name
        self.step_duration = float(step_duration)
        self.max_episode_steps = int(max_episode_steps)
        self.hover_height = float(hover_height)
        self.target_radius = float(target_radius)
        self.success_radius = float(success_radius)
        self.min_target_radius = float(min_target_radius)
        self.world_radius = float(world_radius)
        self.max_velocity_enu = float(max_velocity_enu)
        self.max_yaw_rate_deg = float(max_yaw_rate_deg)
        self.rpc_timeout = float(rpc_timeout)
        self.reset_mode = reset_mode
        self.arm_timeout = float(arm_timeout)
        self.camera_name = camera_name
        self.image_type = image_type
        self.image_shape = (int(image_shape[0]), int(image_shape[1])) if camera_name else None

        self.client = client if client is not None else airsim.MultirotorClient(
            ip=host, port=port, timeout_value=rpc_timeout)
        self._owns_client = client is None

        self.action_space = spaces.Box(
            low=np.array([-self.max_velocity_enu] * 3 + [-self.max_yaw_rate_deg], dtype=np.float32),
            high=np.array([self.max_velocity_enu] * 3 + [self.max_yaw_rate_deg], dtype=np.float32),
            dtype=np.float32)
        self.state_space = spaces.Box(
            low=np.array([-self.world_radius] * 3 + [-50.0] * 3 + [-1.0] * 4 + [-20.0] * 3, dtype=np.float32),
            high=np.array([self.world_radius] * 3 + [50.0] * 3 + [1.0] * 4 + [20.0] * 3, dtype=np.float32),
            dtype=np.float32)
        if self.camera_name is None:
            self.observation_space = self.state_space
        else:
            # RGB uint8 frames, so Stable-Baselines3 can wrap the environment
            # with its own frame stacking and a CNN policy.
            self.observation_space = spaces.Dict({
                'state': self.state_space,
                'image': spaces.Box(low=0, high=255,
                                    shape=self.image_shape + (3,), dtype=np.uint8)})

        self._steps = 0
        self._previous_distance = 0.0
        self.target_enu = np.zeros(3, dtype=np.float64)
        self.target_offset_enu = np.zeros(3, dtype=np.float64)
        self._last_collision_timestamp = 0
        self._airborne = False
        self._was_airborne = False

    # ------------------------------------------------------------- internals
    def _state(self):
        return self.client.getMultirotorState(vehicle_name=self.vehicle)

    def _position_enu(self, state):
        """True position in ENU, never clipped to the observation space."""
        k = state.kinematics_estimated
        return np.asarray(
            frames.ned_to_enu_vector(k.position.x_val, k.position.y_val, k.position.z_val),
            dtype=np.float64)

    def _observe_state(self, state):
        k = state.kinematics_estimated
        east, north, up = frames.ned_to_enu_vector(k.position.x_val, k.position.y_val, k.position.z_val)
        v_east, v_north, v_up = frames.ned_to_enu_vector(
            k.linear_velocity.x_val, k.linear_velocity.y_val, k.linear_velocity.z_val)
        qw, qx, qy, qz = frames.ned_quaternion_to_enu(
            k.orientation.w_val, k.orientation.x_val, k.orientation.y_val, k.orientation.z_val)
        ax, ay, az = frames.frd_to_flu_vector(
            k.angular_velocity.x_val, k.angular_velocity.y_val, k.angular_velocity.z_val)
        observation = np.array(
            [east, north, up, v_east, v_north, v_up, qw, qx, qy, qz, ax, ay, az],
            dtype=np.float32)
        # A step can carry the vehicle past world_radius before the
        # out-of-bounds check ends the episode, leaving the final observation
        # outside the declared Box. Clipping keeps every returned observation
        # inside the space; the unclipped position stays in info.
        return np.clip(observation, self.state_space.low, self.state_space.high)

    def _observe_image(self):
        """One RGB frame from the vehicle camera, resized to image_shape.

        AirSim returns raw bytes with ``compress=False`` in B, G, R order, so
        there is no PNG to decode; the frame is reordered to RGB here because
        that is what image models and the Gymnasium convention expect.
        """
        request = airsim.ImageRequest(self.camera_name, self.image_type,
                                      pixels_as_float=False, compress=False)
        response = self.client.simGetImages([request], vehicle_name=self.vehicle)[0]
        height, width = int(response.height), int(response.width)
        raw = response.image_data_uint8
        # msgpack delivers bytes; np.asarray() would try to parse those as a
        # numeric string literal, so use frombuffer for byte payloads.
        data = (np.frombuffer(raw, dtype=np.uint8) if isinstance(raw, (bytes, bytearray))
                else np.asarray(raw, dtype=np.uint8).reshape(-1))
        if height <= 0 or width <= 0 or data.size != height * width * 3:
            raise RuntimeError('camera %r returned no usable frame: %dx%d, %d bytes'
                               % (self.camera_name, width, height, data.size))
        bgr = data.reshape(height, width, 3)
        target_h, target_w = self.image_shape
        if (height, width) != (target_h, target_w):
            if cv2 is not None:
                bgr = cv2.resize(bgr, (target_w, target_h), interpolation=cv2.INTER_AREA)
            else:  # pragma: no cover - opencv is a declared dependency
                rows = np.linspace(0, height, target_h + 1).astype(int)
                cols = np.linspace(0, width, target_w + 1).astype(int)
                bgr = np.stack([bgr[rows[i]:rows[i + 1], cols[j]:cols[j + 1]].mean(axis=(0, 1))
                                for i in range(target_h) for j in range(target_w)])
                bgr = bgr.reshape(target_h, target_w, 3).astype(np.uint8)
        return np.ascontiguousarray(bgr[:, :, ::-1])

    def _observe(self, state):
        state_observation = self._observe_state(state)
        if self.camera_name is None:
            return state_observation
        return {'state': state_observation, 'image': self._observe_image()}

    def _wait_until_ready(self, timeout):
        import time
        deadline = time.monotonic() + timeout
        last_error = None
        while time.monotonic() < deadline:
            try:
                return self._state()
            except Exception as error:  # the simulator may not have the vehicle yet
                last_error = error
                time.sleep(0.25)
        raise TimeoutError('simulator did not report a vehicle within %.1fs: %r' % (timeout, last_error))

    def _collision(self, state):
        info = self.client.simGetCollisionInfo(vehicle_name=self.vehicle)
        collided = bool(info.has_collided) and int(info.time_stamp) > self._last_collision_timestamp
        self._last_collision_timestamp = max(self._last_collision_timestamp, int(info.time_stamp))
        # Ground contact while the vehicle is down is not a crash: firmware-
        # backed vehicles sit on (and push into) the ground during spin-up and
        # touchdown, which is reported as a collision. Only count contacts
        # while flying, or when this step began airborne: an impact can make a
        # firmware vehicle report Landed in the same step, and that event must
        # not be discarded.
        flying = int(state.landed_state) == int(airsim.LandedState.Flying)
        if not flying and not self._was_airborne:
            return False
        return collided

    def _info(self, state, observation, distance, collision, success, out_of_bounds):
        k = state.kinematics_estimated
        return {
            'timestamp': int(state.timestamp),
            'landed_state': int(state.landed_state),
            'position_ned': [k.position.x_val, k.position.y_val, k.position.z_val],
            'distance_m': float(distance),
            'collision': bool(collision),
            'success': bool(success),
            'out_of_bounds': bool(out_of_bounds),
            'steps': self._steps,
            'target_enu': self.target_enu.tolist(),
            'target_offset_enu': self.target_offset_enu.tolist(),
        }

    def _hold(self):
        """Stop a finished episode without dropping control while airborne."""
        try:
            self.client.hoverAsync(vehicle_name=self.vehicle).get(timeout=self.rpc_timeout)
        except Exception:
            # Best effort: close() still lands the vehicle.
            pass

    def _land_and_disarm(self):
        """Put the vehicle down before the next reset teleports it."""
        import time
        try:
            self.client.landAsync(timeout_sec=30, vehicle_name=self.vehicle).get(
                timeout=self.rpc_timeout + 35)
            self._airborne = False
        except Exception:
            # Fall through to the arm retry, which reports a clear error.
            return
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            try:
                if self.client.getMultirotorState(vehicle_name=self.vehicle).landed_state == \
                        airsim.LandedState.Landed:
                    break
            except Exception:
                break
            time.sleep(0.25)
        try:
            self.client.armDisarm(False, self.vehicle)
        except Exception:
            pass

    def _climb_from_ground(self):
        """Reach the hover height without a 'takeoff' command.

        PX4 converts a takeoff command into an altitude above its home point
        and refuses it with "Already higher than takeoff altitude" when the
        estimator's local origin has drifted below the vehicle since the
        previous landing (routine once GPS noise is modelled). An Offboard
        position setpoint relative to the current estimate has no such
        absolute reference and works from any drift.
        """
        import time
        position = self._position_enu(self._state())
        start_up = float(position[2])
        # ENU target: same place, hover_height above the current estimate.
        target_enu = (position[0], position[1], position[2] + self.hover_height)
        target_up = target_enu[2]
        north_cmd, east_cmd, down_cmd = frames.enu_to_ned_vector(*target_enu)
        deadline = time.monotonic() + self.rpc_timeout + 25
        result = self.client.moveToPositionAsync(north_cmd, east_cmd, down_cmd, self.max_velocity_enu,
                                                 timeout_sec=25, vehicle_name=self.vehicle)
        result.get(timeout=self.rpc_timeout + 30)
        while time.monotonic() < deadline:
            up = float(self._position_enu(self._state())[2])
            if up >= target_up - 0.5:
                break
            time.sleep(0.25)
        else:
            raise RuntimeError('offboard climb did not reach %.1f m (at %.1f m)'
                               % (target_up, up))
        if not self.client.hoverAsync(vehicle_name=self.vehicle).get(timeout=self.rpc_timeout):
            raise RuntimeError('hover was rejected')
        if int(self._state().landed_state) != int(airsim.LandedState.Flying):
            raise RuntimeError('vehicle reports landed after the climb')
        # The climb target came from the drifting local estimate; confirm with
        # ground truth that the vehicle actually left the ground.
        truth_up = -self.client.simGetGroundTruthKinematics(
            vehicle_name=self.vehicle).position.z_val
        if truth_up - start_up < 0.5 * self.hover_height:
            raise RuntimeError('ground truth shows no climb: %.2f m -> %.2f m'
                               % (start_up, truth_up))

    def _arm_with_retry(self):
        """Wait for the controller to accept arming.

        A firmware-backed vehicle refuses to arm while its estimator is still
        converging or while a failsafe is active, so a single attempt turns a
        temporary condition into a failed episode.
        """
        import time
        deadline = time.monotonic() + self.arm_timeout
        attempt = 0
        while True:
            try:
                if self.client.armDisarm(True, self.vehicle):
                    return
            except Exception:
                # PX4 raises instead of returning False while it has no home
                # position or its ground estimate is not stable yet.
                pass
            if time.monotonic() >= deadline:
                raise RuntimeError('vehicle did not accept arming within %.0fs'
                                   % self.arm_timeout)
            attempt += 1
            if attempt % 4 == 3:
                # PX4 refuses to arm in Position mode after a landing when no
                # RC is configured; asking for Hold moves it to a mode where
                # arming is allowed again.
                try:
                    self.client.hoverAsync(vehicle_name=self.vehicle).get(
                        timeout=self.rpc_timeout)
                except Exception:
                    pass
            time.sleep(0.5)

    # ------------------------------------------------------------ gymnasium
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        options = options or {}
        self._steps = 0

        # A firmware-backed vehicle cannot be teleported: PX4 sees an
        # impossible position jump (or, after a teleport to the ground,
        # rejects takeoff with "already higher than takeoff altitude").
        # reset_mode='land' therefore puts the vehicle down and takes off
        # again from wherever it is; the goal is sampled relative to the new
        # hover pose, so a drifted start is harmless.
        if self.reset_mode == 'land':
            if self._airborne:
                self._land_and_disarm()
        else:
            self.client.reset()
        self._wait_until_ready(timeout=20.0)
        self.client.enableApiControl(True, self.vehicle)
        self._arm_with_retry()
        if self.reset_mode == 'land':
            # Mark airborne before the climb: if it fails midway the vehicle
            # may already be off the ground, and close() must still land it.
            self._airborne = True
            self._climb_from_ground()
        else:
            if not self.client.takeoffAsync(timeout_sec=20, vehicle_name=self.vehicle).get(timeout=self.rpc_timeout + 25):
                raise RuntimeError('takeoff was rejected or timed out')
            if not self.client.hoverAsync(vehicle_name=self.vehicle).get(timeout=self.rpc_timeout):
                raise RuntimeError('hover was rejected')
        self._airborne = True

        state = self._state()
        observation = self._observe(state)
        position = self._position_enu(state)
        # Ignore collisions that happened before this episode (takeoff scrape,
        # the previous landing): baseline on the current counter.
        self._last_collision_timestamp = int(
            self.client.simGetCollisionInfo(vehicle_name=self.vehicle).time_stamp)
        target = options.get('target_enu')
        if target is None:
            angle = self.np_random.uniform(0.0, 2.0 * math.pi)
            # Uniform over the disc between min_target_radius and
            # target_radius: sqrt avoids clumping near the inner edge.
            span = self.target_radius - self.min_target_radius
            radius = self.min_target_radius + span * math.sqrt(self.np_random.uniform(0.0, 1.0))
            offset = np.array([radius * math.cos(angle), radius * math.sin(angle), 0.0])
        else:
            offset = np.asarray(target, dtype=np.float64) - position
        # Goals are relative to the hover pose reached by this reset; hover
        # drift between resets makes absolute coordinates differ slightly, so
        # the offset is what a seed actually determines.
        self.target_offset_enu = offset
        self.target_enu = position + offset
        distance = float(np.linalg.norm(self.target_enu - position))
        self._previous_distance = distance
        info = self._info(state, observation, distance, False, False, False)
        return observation, info

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        if action.shape != self.action_space.shape:
            raise ValueError('action shape %r does not match %r' % (action.shape, self.action_space.shape))
        action = np.clip(action, self.action_space.low, self.action_space.high)
        v_east, v_north, v_up = (float(value) for value in action[:3])
        # ROS yaw rate is CCW about up; AirSim yaw rate is CW about down.
        yaw_rate_ned = -float(action[3])
        north, east, down = frames.enu_to_ned_vector(v_east, v_north, v_up)
        finished = self.client.moveByVelocityAsync(north, east, down, self.step_duration,
                                                   yaw_mode=airsim.YawMode(True, yaw_rate_ned),
                                                   vehicle_name=self.vehicle)
        finished.get(timeout=self.rpc_timeout + self.step_duration)

        state = self._state()
        observation = self._observe(state)
        self._steps += 1
        # Distance and the bounds check use the true (unclipped) position.
        position = self._position_enu(state)
        distance = float(np.linalg.norm(self.target_enu - position))
        # _was_airborne still holds the previous step's state here, so an
        # impact that makes the vehicle report Landed within this step is not
        # mistaken for sitting on the ground.
        collision = self._collision(state)
        self._was_airborne = int(state.landed_state) == int(airsim.LandedState.Flying)
        out_of_bounds = float(np.linalg.norm(position[[0, 1]])) > self.world_radius
        success = distance <= self.success_radius

        reward = float(self._previous_distance - distance)
        self._previous_distance = distance
        if success:
            # No time penalty on the step that reaches the goal, so success is
            # always worth the full bonus plus the progress made.
            reward += 10.0
        else:
            reward -= 0.01
        if collision:
            reward -= 10.0
        if out_of_bounds:
            reward -= 10.0

        terminated = bool(success or collision or out_of_bounds)
        truncated = self._steps >= self.max_episode_steps and not terminated
        if terminated or truncated:
            self._hold()
        info = self._info(state, observation, distance, collision, success, out_of_bounds)
        return observation, reward, terminated, truncated, info

    def close(self):
        if self.client is None:
            return
        if self._airborne:
            try:
                self.client.landAsync(timeout_sec=30, vehicle_name=self.vehicle).get(timeout=self.rpc_timeout + 35)
                if self.client.getMultirotorState(vehicle_name=self.vehicle).landed_state == airsim.LandedState.Landed:
                    self.client.armDisarm(False, self.vehicle)
            except Exception:
                # Leave control enabled rather than dropping the vehicle mid-air.
                self._airborne = False
                if self._owns_client:
                    self.client.close()
                self.client = None
                return
            self._airborne = False
        try:
            self.client.enableApiControl(False, self.vehicle)
        except Exception:
            pass
        if self._owns_client:
            self.client.close()
        self.client = None
