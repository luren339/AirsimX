# Configurable LiDAR direction tables

The Unreal LiDAR can optionally load an ordered CSV direction table. The
default remains the existing rotating, multi-channel sensor with a 10 Hz
update rate. `import airsim`, the LiDAR RPC response and coordinate frames
are unchanged.

| Setting | Default | Meaning |
| --- | --- | --- |
| `UpdateFrequency` | `10` | Requested sensor updates per simulation second; finite and positive. Actual output also depends on the simulation tick and ray-tracing load. |
| `ScanPatternFile` | empty | CSV filename, resolved by the simulator process. Use an absolute path for reproducibility. |
| `ScanPatternMode` | `sequential` | `sequential` or `full_fov_uniform`; a non-default mode requires a CSV. |

With no CSV, channel count, rotation frequency and the original ray generation
remain in effect. A CSV replaces that ray-direction generation; channel count
and rotation frequency do not describe the loaded pattern. Range, configured
horizontal/vertical FOV, collision ray tracing and output frames still apply.
Horizontal FOV is a directed interval: `359` to `0` is a narrow cross-zero
window, while `0` to `360` spans a full circle. The original `0` to `359`
default continues to exclude the final degree.

## CSV format and sampling

Use exactly three unquoted columns: `Time/s,Azimuth/deg,Zenith/deg` (the shorter
headers `Time,Azimuth,Zenith` are also accepted). LF/CRLF line endings and
surrounding whitespace are supported. Time and angles must be finite numeric
values. Zenith is in `[0,180]` degrees from +Z. Elevation is `90 - zenith`;
azimuth wraps at 360 degrees. The CSV convention uses positive azimuth towards
the left; the Unreal ray generator reverses it for AirSim's NED convention.
Malformed rows, empty tables and unknown modes fail at sensor construction.

- `sequential` consumes successive entries, wraps at the end, and requests
  approximately `PointsPerSecond × elapsed simulation time` rays per update.
- `full_fov_uniform` requests `round(PointsPerSecond / UpdateFrequency)` rays
  (at least one), evenly spans the full table including its endpoints, and
  repeats this selection each frame. If there are more rays than entries,
  directions repeat. This stabilizes directional coverage for snapshot-based
  estimation experiments; it does not synthesize a moving physical scanner.

Both modes retain the 100,000-ray cap per update. FOV filtering and missed
collisions reduce the number of returned XYZ points below the requested rays.
The time column preserves the table's row format only: it does **not** set
per-ray acquisition times or the output frame timestamp. All rays in an update
use the supplied vehicle pose. No per-point time, motion distortion, intensity,
return model or hardware noise is added. A vendor direction file does not make
this sensor equivalent to a MID-360/MID-360S.

## Small, independently authored example

`tools/airsimx/settings/synthetic-scan-pattern.csv` contains 12 synthetic
directions, without third-party sensor data. In
`tools/airsimx/settings/lidar-scan-pattern.json`, replace `ScanPatternFile`
with its absolute filename and launch the rebuilt simulator with that settings
file. It requests 12 rays at 20 Hz. This sparse example demonstrates the
configuration, not a usable SLAM scan pattern. Supply an appropriately licensed
table and an explicit rate/coverage configuration for an experiment.

Read `getLidarData('Lidar1', 'Drone1')` without arming the vehicle. Count unique
source timestamps over a simulation-time interval, inspect point counts and
coverage, and record the exact simulator build. Client polling frequency is
separate from sensor update frequency. In particular, a fast poll can repeatedly
return the same frame. Compare a default settings run with a CSV run using the
same scene and vehicle pose.

## Tests and validation limits

`AirLibUnitTests --test LidarSettings` checks default/explicit settings and
invalid update rates. `AirLibUnitTests --test LidarScanPattern` checks CSV
validation, angle conversion, wraparound, sampling indices and sample budgets.
Both are included in the normal native test executable and Windows core CI.
The Python offline suite does not compile or ray-trace the Unreal plugin.
Native tests do not establish simulator output frequency, moving-scan fidelity
or SLAM accuracy; those require a separately recorded simulator experiment.
