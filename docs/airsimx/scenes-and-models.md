# Procedural scenes and visual models / 程序化场景与外观模型

These tools migrate an existing utility-gallery layout, GLB geometry builder,
approximate quadrotor/camera/mount generators and their UE editor scripts.
They generate visual geometry. They do not calibrate a vehicle, model a sensor's
optics or scanning pattern, or establish safe flight clearance in Unreal.
No vendor CAD, reference photograph, SubT texture, `.uasset` or `.umap` is included.

## Generate without Unreal or external assets

Use Python 3.12 from the repository root. Scene generation uses NumPy and OpenCV,
already pinned by the existing offline CI requirements. The OBJ generators and
UE workflow plans use only the standard library; Pillow and OS fonts are not
required.

```bash
python -m pip install -r PythonClient/requirements-v01-windows.txt -r PythonClient/requirements-gym.txt
python tools/airsimx/scenes/generate_utility_gallery_glb.py
python tools/airsimx/models/generate_open_quadrotor_frame.py
python tools/airsimx/models/generate_sensor_rig_meshes.py
```

Default outputs are resolved relative to this checkout, regardless of the current
working directory. They live under the ignored `build/airsimx/` tree:

| Output | Contents |
| --- | --- |
| `scenes/utility_gallery_preview.glb` | One mesh with embedded PNGs and PBR material groups |
| `scenes/utility_gallery_geometry.json` | SHA-256, source-frame bounds, topology, services, lights and fixture checks |
| `scenes/*.png` | Code-generated concrete PBR maps and the retained signage atlas |
| `models/frame/` | `OpenQuadrotorFrame30cm.obj`, MTL and measured manifest |
| `models/camera/` | `approximate_camera_shell.obj`, MTL and measured manifest |
| `models/lidar_mount/` | `mid360s_mount_plate.obj`, MTL and measured manifest |

Every generator accepts `--output-dir`. The scene also accepts `--config`,
defaulting to the adjacent `scenes/settings/utility_gallery.json`. For example:

```bash
python tools/airsimx/scenes/generate_utility_gallery_glb.py --config tools/airsimx/scenes/settings/utility_gallery.json --output-dir build/airsimx/scenes/example
python tools/airsimx/models/generate_sensor_rig_meshes.py --output-dir build/airsimx/models/example
python tools/airsimx/models/generate_open_quadrotor_frame.py --output-dir build/airsimx/models/example/frame
```

The default scene retains the existing 3×3 junction graph: nine junctions, twelve
connecting corridors, four independent loops and two 10 m dead-end service spurs.
Corridors are 4 m wide and 3.6 m high; junctions are 5 m wide and 3.6 m high;
spurs are 3 m wide and 3.3 m high. Uneven cell lengths, pipes, cable trays,
supports, drains, structural joints, lights and concealed portal-return routes
are retained. The current source does not place fixed cabinets, inspection
hatches or sign face planes; unused material definitions/atlas remain compatible
with the original exporter.

With the supplied settings, the generated mesh has 299,376 triangles and bounds
`[-40.75, -24.75, -0.25]` to `[45.75, 27.75, 3.85]` m in the layout frame.
The configured central envelope is 2 m wide, 2.2 m high and centered at 1.8 m.
Its analytic fixture checks report 2,383 checked placements, no violations,
0.210 m minimum lateral margin and 0.277 m minimum vertical margin. These are
checks of generated placement records, not swept-volume collision queries or
flight/sensor acceptance. Changing the settings requires checking the new output.
Finite, positive dimensions/spacing and envelope bounds are validated before
generation.

The layout uses meters and Z-up coordinates. GLB positions and normals use the
proper rotation `(x, z, -y)` into glTF Y-up coordinates. UE preview light/start
placement retains the original importer convention `(100x, -100y, 100z)` cm.
Check the imported orientation and dimensions for your engine/importer version.

## Approximate UAV visuals

OBJ coordinates are centimeters, +X forward/optical direction and +Z up. The
manifests calculate their bounds from the generated vertices:

| Model | Full mesh extent X × Y × Z (cm) | Boundary |
| --- | --- | --- |
| Open quadrotor frame | 30.000 × 22.200 × 9.270 | Includes skids and sensor supports; no propeller meshes |
| Camera shell | 4.005 × 2.900 × 2.180 | Illustrative housing; origin at the lens front plane |
| Mount and rails | 13.700 × 10.000 × 1.670 | Includes posts; central plate area is 10 × 10 cm |

“30 cm” describes the approximate visible frame length. It is not an exact
vehicle scale, rotor diagonal, manufacturer drawing, collision envelope or
dynamic model. The camera's reported 2.8 mm/112°/60 fps reference values do not
validate the generated housing or configure image acquisition. No LiDAR body CAD
is included. Mass, inertia, thrust, inherited AirSim vehicle scale and collision
remain separate from these visual meshes.

## Optional external inputs

The scene's default PBR maps are deterministic code-generated matte textures.
`--texture-dir DIR` explicitly enables the original atlas-cropping route. It
requires `shaft_albedo.png`, `shaft_normal.png` and `shaft_roughness.png`, each at
least 852×662 pixels. The original `(340,150)-(852,662)` concrete-panel crop,
mirrored-repeat sampling, roughness floor of 225/255 and glTF G/B packing are
retained. Missing, unreadable or undersized inputs fail instead of silently
falling back. The manifest lists exactly the three supplied files.

Obtain and check permission for any external texture separately before using or
redistributing it. The private reference atlases used by the original workspace
are not supplied and their redistribution permission is unconfirmed. Defaults
do not search that workspace or any reference directory.

`models/generate_uav_sensor_rig_mesh_data.py` preserves the existing Datasmith
transform-to-C++ schema. It needs an explicit JSON input; it does not include
meshes or modify the AirSim plugin. The input has `mesh_actor_instances` records
containing `static_mesh_asset`, `translation` (cm), `rotation_xyzw` (unit
quaternion) and `scale`. The original assembly expected 13 records; an explicit
`--expected-count` supports another externally supplied assembly.

```bash
python tools/airsimx/models/generate_uav_sensor_rig_mesh_data.py path/to/transforms.json --output build/airsimx/models/UAVSensorRigMeshData.h
```

The emitted compatibility header has no consumer added to the upstream Pawn by
this contribution. External CAD/assets must be imported and licensed separately.

## UE editor workflows

Use a separate UE project and new `/Game/...` output paths. Enable Python editor
scripting, Editor Scripting Utilities and the GLB/Interchange importer. These
scripts need the **full Editor**; the Python commandlet may not initialize
`StaticMeshEditorSubsystem`. A missing subsystem is an error, not a zero collision
count. The public editor API used for visual assembly is
[SubobjectDataSubsystem](https://dev.epicgames.com/documentation/en-us/unreal-engine/python-api/class/SubobjectDataSubsystem?application_version=5.4).

All four entry points support `--dry-run` with ordinary Python. A dry run validates
inputs and writes a plan with `engine_executed: false`; it does not import `unreal`.
Use absolute file paths when running from the Editor. Example plan commands below
assume the default generated files and an existing `PROJECT_DIR`:

```bash
python tools/airsimx/scenes/build_utility_gallery_preview.py --project-dir PROJECT_DIR --map-asset /Game/MyGallery/Maps/Preview --asset-root /Game/MyGallery/Generated --report build/airsimx/scenes/ue-build-plan.json --dry-run
python tools/airsimx/models/import_generated_sensor_meshes.py --asset-root /Game/MyDrone --report build/airsimx/models/import-plan.json --dry-run
python tools/airsimx/models/create_uav_sensor_rig.py --asset-root /Game/MyDrone --blueprint-asset /Game/MyDrone/BP_Rig --parent-blueprint /AirSim/Blueprints/BP_FlyingPawn --report build/airsimx/models/rig-plan.json --dry-run
```

To execute, pass the script and its arguments to the Editor's
`-ExecutePythonScript="ABSOLUTE_SCRIPT_PATH arguments..."`, without `--dry-run`.
For example (replace every placeholder and quote paths containing spaces):

```text
UnrealEditor PROJECT.uproject -unattended -ExecutePythonScript="ABSOLUTE_REPO/tools/airsimx/scenes/build_utility_gallery_preview.py --project-dir ABSOLUTE_PROJECT_DIR --glb ABSOLUTE_REPO/build/airsimx/scenes/utility_gallery_preview.glb --metadata ABSOLUTE_REPO/build/airsimx/scenes/utility_gallery_geometry.json --map-asset /Game/MyGallery/Maps/Preview --asset-root /Game/MyGallery/Generated --report ABSOLUTE_REPO/build/airsimx/scenes/ue-build.json"
```

The gallery builder verifies the GLB SHA against its geometry manifest, then
imports one StaticMesh, removes both simple and convex imported collision,
sets `UseComplexAsSimple`, places lights and a PlayerStart and saves the chosen
level. It hashes every other existing project `.umap` before and after the build.
It checks that `--project-dir` is the active UE project. Preserve its JSON report,
then run `scenes/validate_utility_gallery_preview.py` in the Editor with
`--project-dir`, the same `--map-asset`, `--build-report` and a separate `--report`.
The validator reopens the map and checks actor/light/start counts, bounds,
material groups, collision configuration and protected-map hashes. Its dry run
only validates the saved build report and filesystem hashes.

`models/import_generated_sensor_meshes.py --generated-dir DIR --asset-root /Game/MyDrone`
imports the camera, mount and frame into `Camera`, `Mounts` and `Models` subfolders.
Its report records engine version, input SHA-256, asset paths and imported bounds.
`models/create_uav_sensor_rig.py` creates a child of `--parent-blueprint` with
three movable, `NoCollision` visual components from
`models/settings/sensor_rig.json` (or `--config`). Locations are cm and rotations
are roll/pitch/yaw degrees. An explicit `asset` in a custom component record can
reference an already imported external mesh instead of an `asset_relative_path`.
The supplied rig adds no third-party LiDAR body and changes no camera/sensor
extrinsics, propeller positions, inherited body visibility, scale or dynamics.
Inspect the result with your parent Pawn; these visual placements are examples.

Existing output assets are refused by default. `--replace-existing` explicitly
updates the selected generated output. For the gallery, reserve the chosen map
and asset root for this tool; only `UtilityGallery_` actors are replaced in an
existing target level. For rig Blueprints, replacement additionally requires the
script's metadata marker and unchanged parent/component-name set; it updates the
same components without adding duplicates. A different parent/name set needs a
new Blueprint path. An unrelated Blueprint is never overwritten. Import/build
errors are raised; an unsuccessful run may leave partially generated assets in
the selected output folders, so retain logs before retrying.

## Verification and evidence boundary

```bash
python -m unittest discover -s tools/airsimx/tests -v
python -I tools/airsimx/ci/run_offline.py --output build/airsimx/ci/offline
```

The migrated tests execute generation in temporary directories, check topology,
clearance records, every GLB primitive's finite coordinates/normals/UVs, valid
indices, nonzero triangle areas and winding, embedded PNGs/materials, OBJ face
indices/areas and measured dimensions. They also exercise external-texture errors,
external transform validation, CLI plans, protected-map hashes and controlled
collision API failure cases. The existing offline CI discovers these `unittest`
cases without Unreal, ROS, vendor assets, Pillow or extra optional-test skips.

An offline pass is not UE import acceptance. UE reports must identify the actual
engine version and separately show import, save/reopen and component results.
Even a successful static UE check does not prove rendering quality, collision
queries, sensor rates/timing, AirSim flight, SLAM accuracy or hardware equivalence.

The 2026-11-24 local editor checks used UE **5.8.2** in a separate project that
reused an existing **private prebuilt AirSim plugin and Blueprint assets**. The
new GLB and three OBJ meshes were imported; the gallery was saved/reopened with
73 lights, 12 used material slots and zero simple/convex hulls; the derived visual
Pawn was created in a separate successful Editor process. This does not validate
a build of the upstream plugin. The first combined NullRHI run crashed during
shutdown after writing its static asset reports; keep those step reports separate
from the host process exit status. No third-party binary or asset is distributed
by this contribution, and no rendering/flight claim follows from these checks.
