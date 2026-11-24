# PX4 in software / 软件在环

Use this path when you need the PX4 estimator and controller inside a Blocks experiment. For client development alone, start with [SimpleFlight](windows-development.md). These instructions never require a USB flight controller or firmware flashing.

## Build the pinned target

Use an isolated Ubuntu 24.04 user environment with the native PX4 build prerequisites: Git, GCC/G++, CMake, Ninja, make, ccache, Python 3 with venv/pip, Eigen and XML development tools. PX4 sources and Python dependencies are prepared separately from AirsimX. Run as a regular user in the AirsimX checkout:

```bash
export AIRSIMX_PX4_ROOT="$HOME/airsimx-sitl"
bash tools/airsimx/px4/build_sitl.sh
```

The script verifies the PX4 v1.17.0 commit in [environment.lock.json](../../tools/airsimx/px4/environment.lock.json) and builds a target with `simulator_mavlink`. It uses its own install root. Do not point it at an existing PX4 development checkout. A successful build alone does not establish a simulator connection.

## Connect to Blocks

1. Close other instances of **this simulation** that own its ports. Choose the [single PX4 profile](../../tools/airsimx/px4/settings-remote-px4.json). The public examples use **loopback**, even where historical filenames contain `remote`. Supply the selected file to your Blocks session with the AirSim `-settings=<absolute JSON path>` launch argument. Do not overwrite an unrelated user's settings.
2. Start the Blocks scene and wait for the simulator log to report that it is waiting for TCP port **4560**. With a same-host setup that shares loopback, start PX4 from a second shell using the same `AIRSIMX_PX4_ROOT`:

   ```bash
   bash tools/airsimx/px4/start_sitl.sh run-001
   ```

   The run identifier must be new. The script writes a separate console log and run directory. Do **not** probe the already-listening simulator port with an extra TCP client: that can consume its single connection.
3. Use the Python client with RPC port **41452** and vehicle name `PX4SITL`. `getMultirotorState()` is read-only; a new, advancing timestamp is stronger evidence than a successful TCP connect.

For two machines, copy the example first and set the simulator bind interface, peer `ControlIp`, and `PX4_SIM_HOST_ADDR` to **your** private endpoints. Keep RPC on loopback or use a private tunnel. Do not expose RPC or simulator sockets to the internet; this guide does not change a firewall, WSL configuration or global network settings. WSL NAT does not in general share loopback with Windows.

## What the links do

| Link | Role |
| --- | --- |
| AirSim RPC | Reads simulator state, images and sensors for clients. |
| Simulator MAVLink | Connects the virtual sensors and actuators to the PX4 SITL process. |
| PX4 DDS (optional) | Exposes PX4 messages on a ROS graph; requires a separately built DDS-enabled board, compatible `px4_msgs` and an agent. The default MAVLink-only build does not supply this. |

Two- and three-vehicle profiles allocate separate instances/ports. The [validation table](validation.md) distinguishes dual lockstep from triple free-running results. Calibrated examples vary GPS accuracy with satellite geometry and model IMU scale, misalignment and nonlinearity. These are simulation profiles, not calibrations for physical hardware.

中文提示：同机示例默认只用回环；跨机器时自己复制并填写私网端点。先启动 UE 等待连接，再启动软件飞控。RPC、仿真 MAVLink 和 DDS 是三条不同的链路，不能用某一条的连接成功代替另外两条的验证。

## Optional FAST-LIO2 external odometry / 可选外部里程计

The `airsim_nav_bridge` package also contains `fastlio_to_px4_odometry`, migrated from an existing FAST-LIO2 SITL bridge. It consumes **estimator output**, `/clock` and PX4 `SensorCombined` timestamps, then publishes `px4_msgs/VehicleOdometry`. It does not read AirSim truth, GNSS, arming state or control setpoints. The PX4 topic name `vehicle_visual_odometry` identifies the external-estimate input; it does not imply that the estimate came from a camera.

This executable is optional. The default MAVLink-only build above, the general `airsim_ros2` package and the existing navigation source/recorder do not require `px4_msgs` or a SLAM workspace. To run this example, supply a separate **PX4 v1.17.0 DDS-enabled SITL**, **`px4_msgs` `release/1.17`** built for **ROS 2 Jazzy**, and **Micro XRCE-DDS Agent v2.4.3**. These external projects and FAST-LIO2 are not vendored or installed by AirsimX. The conditional `package.xml` dependency is enabled by `AIRSIMX_WITH_PX4_ODOMETRY=1`; leave it unset for the default package. Message versions and DDS topic names must match the selected firmware. Keep the agent and SITL on the local host for this example, with no USB/physical flight controller attached.

From the repository root, with `PX4_MSGS_WS` set to your separately prepared workspace:

```bash
source /opt/ros/jazzy/setup.bash
source "$PX4_MSGS_WS/install/setup.bash"
export AIRSIMX_WITH_PX4_ODOMETRY=1
colcon --log-base build/airsimx/odom/log build \
  --base-paths ros2/src --packages-select airsim_nav_bridge \
  --build-base build/airsimx/odom/build --install-base build/airsimx/odom/install
source build/airsimx/odom/install/setup.bash
```

Only after checking your local SITL and FAST-LIO2 input frames and time domains, run:

```bash
ros2 run airsim_nav_bridge fastlio_to_px4_odometry --ros-args \
  --params-file "$(ros2 pkg prefix airsim_nav_bridge)/share/airsim_nav_bridge/config/fastlio_to_px4_odometry.yaml"
```

The installed [example configuration](../../ros2/src/airsim_nav_bridge/config/fastlio_to_px4_odometry.yaml) exposes input/output topics, clock topics, covariance floor, startup-position and inter-sample speed limits. It defaults to **pose-only** (`input_has_body_twist: false`), because the migrated FAST-LIO2 producer does not populate `Odometry.twist`. Unset ROS twist fields must not be published as measured zero velocity. Set `input_has_body_twist: true` only for a producer that actually supplies both linear and angular velocity in body FLU. CLI parameter overrides can select distinct per-vehicle topics. This example does not configure or enable PX4 EKF fusion; inspect actual estimator aid-source status separately before making a fusion claim.

### Frame, covariance and time contract

| Input or output | Contract |
| --- | --- |
| Input pose | FAST-LIO2 local world **NWU**, body **FLU**; position is in the local world. This is not an ENU conversion. The generic AirSim ENU odometry output must not be connected directly. Frame names are reported but do not trigger TF lookup or conversion. |
| Output pose | Local **FRD**, arbitrary fixed heading origin (`POSE_FRAME_FRD`), not true-north-aligned NED. Position becomes `(x, -y, -z)`; normalized input quaternion `(x, y, z, w)` becomes Hamilton `(w, x, -y, -z)`, changing both the local and body bases. |
| Twist | Default pose-only output uses `VELOCITY_FRAME_UNKNOWN` and NaN for velocity, angular velocity and velocity variance, as PX4's unknown-data representation. With explicit `input_has_body_twist: true`, both measured FLU vectors become `(x, -y, -z)` and velocity uses `VELOCITY_FRAME_BODY_FRD`. No world-to-body velocity rotation or velocity inference from pose is added. |
| Covariance | Pose position/attitude diagonals, and velocity diagonals only with explicit body twist, are copied. Finite values are clamped to the configured positive floor (default `0.01`); non-finite values are replaced by that floor. Finite positive values exceeding float32 range reject the sample. Cross terms are discarded; this is not general world/body covariance rotation or a calibrated uncertainty model. Unknown twist variance remains NaN, without a confidence floor. |
| Quality and reset | Both remain `0`: quality is unreported, and estimator reset detection/counter propagation is not implemented. |
| Inputs and QoS | `/Odometry` (`nav_msgs/Odometry`), `/clock` and `/fmu/out/sensor_combined` use sensor-data best-effort subscriptions; the output `/fmu/in/vehicle_visual_odometry` publisher is reliable, depth 10. |

The odometry header and `/clock` must share the simulation time domain. The configured PX4 clock source must carry PX4-local HRT microseconds, not an already ROS-synchronized epoch; check firmware time-synchronization settings before use. The bridge waits for positive values from both clocks and maps each sample by:

```text
timestamp_sample = floor(odom_stamp_ns / 1000)
                   + sensor_combined.timestamp - floor(clock_ns / 1000)
timestamp        = max(sensor_combined.timestamp, timestamp_sample)
```

Only positive uint64 timestamps are published. Publication time and mapped `timestamp_sample` must each advance relative to their own last **accepted** values. Advancing PX4 publication time does not allow a mapped sample rollback. The ROS input timestamp must also advance relative to the last accepted input; rejected samples do not move any of these high-water marks, so the same ROS sample can be retried once the mapping advances.

The node refuses non-finite or non-positive `variance_floor`, `max_initial_position_norm_m`, `max_position_speed_m_s` and `diagnostic_period_s` at startup. The floor must also remain positive and finite as float32. Before message assignment, positions, normalized quaternion, explicitly trusted twist and sanitized covariance must fit finite float32 fields; invalid measurements reject the sample instead of overflowing to infinity. The deliberate NaN fields for unknown twist are an exception, not measured data. Startup positions beyond 5 m, inter-sample position speeds above 15 m/s and zero/non-finite quaternions are also rejected under the default configuration. These checks are not a general estimator-health guarantee.

This latest-clock mapping has no clock-pair synchronization, freshness timeout or delay estimation. It does not implement automatic recovery from simulation/PX4 restarts, time rollback, attitude resets or map-optimization jumps; restart the bridge with a fresh estimator run after checking the frames and clocks. A large position jump can be rejected, but an attitude reset or a jump within configured limits is not automatically identified. No physical-hardware or continuous-flight acceptance is claimed.

### Validation commands and limits

The existing offline CI runner now includes `px4-odometry-conversions`, which needs neither ROS nor `px4_msgs`. From the repository root, run it with the same Python environment used for the SDK offline checks:

```bash
python tools/airsimx/ci/run_offline.py --output build/airsimx/ci/offline
```

The explicit ROS suite is separate, under `test/ros` (not recursed by offline `unittest` discovery). With Jazzy and matching `px4_msgs` sourced, use a dedicated ROS domain that is unused by your experiments:

```bash
export ROS_DOMAIN_ID=187
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH="$PWD/PythonClient:$PWD/ros2/src/airsim_nav_bridge:$PYTHONPATH"
python -m unittest discover -s ros2/src/airsim_nav_bridge/test/ros -v
```

The migrated code has been checked with 20 ROS-free helper tests and 24 Jazzy/`px4_msgs` tests covering typed-message serialization, moving pose-only input, explicit body twist, mapped-sample rollback/retry, float32 boundaries, invalid startup settings, synthetic pub/sub and clean node exit. Review corrected the existing bridge's implicit zero twist and added sample-time/numeric boundary guards; these are deliberate protocol/validation changes from the source implementation. This validation used synthetic estimator input in an isolated ROS domain; it did not start UE, an agent or PX4, and does not demonstrate FAST-LIO2 accuracy, PX4 EKF fusion, hardware compatibility or a flight result. The Ubuntu/Windows offline matrix and Windows native gate still need to run on the eventual PR.
