# Validation scope / 哪些结果已经验证

**Rechecked on the independent source export, 2026-11-23:** 66 Python client tests, 64 tool tests and 13 ROS coordinate-conversion tests passed on Windows, with no skips. A separate Windows x64 Release build restored its pinned inputs, built the native core, and passed all 6 AirLib tests. The release check did not cook or fly a new UE package.

The table below describes earlier development runs, not additional tests of this exact export. It is not an all-platform support matrix or a measured improvement over upstream.

| Experiment | Observed result | Boundary |
| --- | --- | --- |
| Windows, UE 5.8.4, DX11, Blocks | SimpleFlight editor and standalone Development package exercised; hover and repeated reset cycles recorded | No public engine binary or packaged environment. No equivalent Linux UE package claim. |
| PX4 v1.17.0, simulated sensors | Single-vehicle lockstep: 52/52 checks. A separate 30-cycle single-vehicle run: 302/302 checks in about 20 minutes | Lockstep pacing is configuration-dependent; not a real-time guarantee. |
| Multiple PX4 instances | Two vehicles in free-running and lockstep configurations, 14/14 each; three free-running, 30/30 in the stricter run | Three-vehicle lockstep and four or more vehicles not validated here. |
| ROS 2 camera bridge | 24/24 checks on SimpleFlight and on PX4, including fresh airborne images; about 10 Hz in the tested PX4 setup | Not a universal bandwidth or FPS figure. |
| Gymnasium / SB3 | State-policy training smoke on PX4 (128 steps); an RGB+state policy trained for 128 steps on SimpleFlight | No claim of PX4 image-policy training or policy convergence. |
| Navigation recording | On an isolated **replacement** map replay, 86 sampled diagnostics matched 10 nonempty resident revisions. Removing producer status left zero witnesses | Not a Blocks flight; not proof of historical waypoint arrival. The external planner and input bags are not shipped. |

The Python client keeps its public import and the simulator keeps the `AirSim` plugin/module name. This does not imply every historical API method, Unity or cars passed these tests. No hardware-in-the-loop or physical aircraft deployment is covered.

For current local evidence run [the offline checks](windows-development.md#run-the-offline-checks). Their JSON report identifies the checkout, package versions, skips and failures. For simulation experiments keep the selected config, timestamps and run-specific records together. Do not publish logs containing private network endpoints or credentials.
