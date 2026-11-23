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
