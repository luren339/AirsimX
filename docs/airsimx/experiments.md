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
