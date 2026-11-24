# Research interfaces / 算法实验接口

## ROS 2 images and telemetry

The Jazzy bridge takes AirSim RPC as input. In a sourced ROS 2 Jazzy environment with the AirsimX Python client installed for the same interpreter, build the included packages in the checkout's `ros2/` directory:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2
colcon build --packages-up-to airsim_ros2
source install/setup.bash
ros2 launch airsim_ros2 airsim_bridge.launch.py host:=127.0.0.1 port:=41451 vehicle_name:=SimpleFlight
```

This example expects a running, locally reachable SimpleFlight simulator. `/<vehicle>/camera_0/image_raw` is raw `bgr8`, capture-stamped, best-effort. Use compatible subscription QoS and check arriving frames, not just discovery. PX4's example RPC port is 41452. Local NED is north/east/down; ENU is east/north/up. World NED→ENU swaps horizontal axes; sensor FRD→FLU does **not**. To compare odometry, pair the two messages by their shared stamp.

## Read-only LiDAR, IMU and camera acquisition

The optional `airsim_sensor_bridge` is migrated from an existing UE5/Jazzy
acquisition bridge. Each enabled sensor has its own RPC client and polling
thread. It does not arm, reset, move, or request vehicle-state odometry.
The original `airsim_bridge` and its command services remain separate.

Start a simulator with the [sensor example](../../tools/airsimx/settings/multisensor-readonly.json)
passed as its `-settings=<absolute path>` argument. This example uses SimpleFlight,
generic spinning LiDAR and an ideal perspective camera; it is not a MID-360S
hardware model or a physical camera calibration. It does not require PX4.
With the client installed for the same Python interpreter as ROS, build and launch:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2
colcon build --packages-up-to airsim_ros2
source install/setup.bash
ros2 launch airsim_ros2 sensor_bridge.launch.py
```

`host`, `port`, `vehicle_name`, `namespace`, `frame_prefix`, `params_file` and
`publish_clock` are launch arguments. A YAML parameter file is installed with
the package. Poll rates are **wall-time RPC rates**, not guaranteed source rates.
Measure unique header stamps and elapsed simulation time separately, particularly
when `ClockSpeed` differs from one. All polling rates and timeouts must be finite
and positive; `publish_camera=false` disables the camera worker.
Failed connections use a separate, interruptible `reconnect_delay_s` (default one
second), not the IMU polling period. Shutdown closes active clients, cancels
outstanding RPC calls and joins their workers before destroying ROS publishers.

| Default topic | Content |
| --- | --- |
| `/uav_a/lidar/points` | Sensor-local FLU `PointCloud2`, XYZ plus zero placeholder intensity |
| `/uav_a/imu` | FLU angular velocity/acceleration; orientation explicitly unavailable |
| `/uav_a/camera/image_raw` | `rgb8` image, using the acquisition code's existing encoding |
| `/uav_a/camera/camera_info` | Same source stamp, optical frame and dimensions as the image |
| `/clock` | Largest accepted source stamp, only if this node is the clock owner |
| `/tf_static` | Configured body-FLU to sensor-FLU / camera-optical extrinsics |

Publishers use reliable QoS with depth 10, compatible with best-effort or reliable
subscribers. Frame IDs are prefixed independently of the ROS namespace; set both
for each vehicle. Exactly one node should publish the global clock. For a second
vehicle that already exists in the simulator, for example:

```bash
ros2 launch airsim_ros2 sensor_bridge.launch.py \
  vehicle_name:=UAV_B namespace:=uav_b frame_prefix:=uav_b publish_clock:=false
```

The LiDAR must explicitly set `DataFrame: SensorLocalFrame` in its vehicle's
`Sensors` configuration. The bridge rejects the inertial-frame default rather
than labelling world points as sensor points. Validation is tied to the SDK's
read-only `connection_generation`, which increases whenever a physical RPC
socket connects. The bridge checks that identity before and after each data RPC
and revalidates settings after a transparent idle reconnect. Settings acquired
across another connection change are discarded rather than used for the sample.
The generation is local connection state, not a server/vehicle identity.
The bridge does not synthesize per-point
time, realistic intensity, or motion compensation. Translations are body-FLU
metres and rotations are sensor-to-body `xyzw` quaternions. The installed YAML
contains an example rig; match those values to your simulator mounts. The Python
node's defaults use co-located sensors, with the usual optical-axis rotation.
The native AirSim IMU reports body measurements: its translation must be zero
and its rotation must be identity. Other IMU mounts are rejected rather than
assigning a misleading frame to unchanged body measurements.

CameraInfo uses the **Scene projection matrix** from `simGetCameraInfo`, rather
than its independent main/CineCamera `fov` field. Supported landscape and square
images follow the Scene capture even when its FOV differs from the main camera. The
principal-point approximation remains `((width-1)/2, (height-1)/2)`. Use a centred,
fixed perspective camera with distortion disabled. Unsupported projection
layouts are rejected; dynamic zoom and physical-camera calibration accuracy are
outside this example's validation.

Portrait sizes (`width < height`) are explicitly unsupported and stop this
acquisition node with a configuration diagnostic. The current RPC and UE Scene
renderer use different axis multipliers for these sizes: at 480x640 and horizontal
FOV90 the renderer uses 240 px focal length while the RPC implies 320 px. The
bridge does not publish that incorrect CameraInfo or silently change calibration.
Camera source timestamps are committed only after both the decoded image and
CameraInfo are ready. A decoding/calibration RPC failure leaves the same timestamp
retryable and does not advance this camera stream's contribution to `/clock`.

The optional `imu_raw_topic` publishes every accepted IMU source sample.
`imu_target_hz_sim_time=0` passes them all; a positive value selects existing
measurements using source-time intervals. This performs no denoising or
interpolation and cannot recover missing measurements or exceed the source rate.
Duplicate and backward source stamps are rejected together. After resetting the
simulator clock, restart **all** acquisition nodes and begin a new recording;
automatic clock-epoch recovery is not implemented. Static mounts and frame/topic
parameters also require a restart when changed.

Sensor headers and `/clock` retain the original source epoch only while it is
representable by ROS Time's signed-int32 seconds. An out-of-range epoch causes
one terminal configuration diagnostic, stops all this node's workers and exits
the CLI with status 1; reconnecting cannot repair it. In particular, multirotor
ScalableClock scales the whole Unix epoch, so ClockSpeed=2 at current dates is
not supported by this bridge. Timestamps are not truncated, wrapped or independently
rebased. Transient transport failures still use the normal reconnect/backoff path.

Offline timestamp/selection tests run with the existing CI entry point. Message
and node tests require a sourced Jazzy environment. The review regressions use
the actual SDK and local sensor-only TCP servers, including idle server replacement,
a dropped calibration response, and an accelerated-epoch node subprocess:

```bash
python3 -m unittest discover -s ros2/src/airsim_ros2/test -p 'test_sensor*.py' -v
```

The command above is run from the repository root with `PythonClient` and
`ros2/src/airsim_ros2` importable (or installed), preserving ROS's existing
`PYTHONPATH`. Tests that need ROS report a skip in the offline Windows/Linux
suite. They do not constitute a simulator flight or a sensor-rate benchmark.

中文提示：此入口只采集传感器，不发布位姿真值或控制指令。外参取自配置；
轮询频率与源采样频率分开统计。相机与CameraInfo配对，仿真重置后重启节点并另开记录。

## Gymnasium and Stable-Baselines3

[Gym environment](../../PythonClient/airsim_gym/envs/quadrotor.py) · [training](../../tools/airsimx/gym/train_sb3.py) · [evaluation](../../tools/airsimx/gym/evaluate_sb3.py)

The environment offers a 13-component state observation; an optional camera adds a `uint8` RGB image. For use from a separate interpreter, install this checkout's Python client and the extras in `PythonClient/requirements-gym.txt` and `PythonClient/requirements-sb3.txt`. The following is a short **simulation** run; unlike the read-only README example, it takes control of the virtual vehicle. Use a dedicated SimpleFlight scene:

```powershell
python tools/airsimx/gym/train_sb3.py --output build/airsimx/gym/first-image-run --total-timesteps 128 --camera 0
```

For PX4 experiments see [the setup guide](px4-sitl.md); use `--reset-mode land` rather than teleporting firmware state. A short training run proves that the interface trains and saves a model, not that the learned policy generalizes.

## Record a search witness

The optional [source node](../../ros2/src/airsim_nav_bridge/airsim_nav_bridge/source_node.py) publishes a simulated clock and sensors. The [capture tool](../../tools/airsimx/nav/sync/capture.py) subscribes to an **existing** compatible navigation graph. The external planner, its custom messages, PX4 DDS workspace and container are **not** distributed here. A plain AirsimX checkout is not enough to run this optional experiment.

In a compatible Linux/Jazzy container: source Jazzy, `px4_msgs` and the planner workspace; use the same UID, domain, IPC and matching QoS as the publishers. Capture before the first map with a new output directory. Pass the actual input parameters/settings and the real container image ID. Example (shell variables denote paths/ID from your own setup):

```bash
python3 tools/airsimx/nav/sync/capture.py --workspace "$NAV_WORKSPACE" --out "$NEW_RUN" --source-config "$NAV_PARAMS" --settings "$AIRSIM_SETTINGS" --image-id "$IMAGE_ID" --wall-seconds 180
```

Ctrl-C or the wall-time limit stops only the recorder. `closed` means the bag finalized, **not** that it contains a witnessed search. Check it offline, with the same message definitions available:

```bash
python3 tools/airsimx/nav/sync/verify.py "$NEW_RUN" --report "$NEW_RUN/check.json"
```

An optional `--expect-goal X Y Z` is only a check. The verifier reconstructs the resident obstacle map from a base and cumulative deltas, compares its occupied-content fingerprint, requires producer status, and joins an accepted sampled search to same-run telemetry. It does not substitute the latest raw revision or a display marker for the planner's map. `search_witnessed` is not a refined solver endpoint, arrival verdict or mission certificate. Missing/ambiguous evidence returns `incomplete`.

中文提示：看图像请核对时间戳和 QoS。训练示例会控制**仿真**载具；录制器只读取已有导航图，两者作用不同。
