# Getting started / 安装与构建

Use **Windows PowerShell** from the repository root. Python-only checks work without Unreal, PX4, ROS or a GPU. The native preview targets Windows x64 / Unreal Engine **5.8.4** / DX11. These are separate setup paths.

## Run the offline checks

```powershell
git clone https://github.com/naipi11/AirsimX.git
cd AirsimX
py -3.12 -m venv .venv
$py = (Resolve-Path .venv/Scripts/python.exe).Path
& $py -m pip install -r PythonClient/requirements-v01-windows.txt -r PythonClient/requirements-gym.txt
& $py -I tools/airsimx/ci/run_offline.py --output build/airsimx/ci/first
```

Read `build/airsimx/ci/first/result.json`: it records suite exit codes, optional skips and the source revision. The three suites cover the client, project tools and ROS coordinate conversions. A pass here is **not** a flight test. Dependencies need a network connection on first installation, or a pre-populated wheelhouse. To import the SDK from outside the checkout, use `& $py -m pip install --no-deps ./PythonClient` in the same environment; installing `airsim` from a public package index does not install this checkout.

## Build Blocks on Windows

Install VS 2022 Build Tools with C++ x64/v143, a Windows SDK and CMake; install UE 5.8.4 separately under Epic's terms. The validated Windows environment used Python 3.12.10 and the VC++ x64 runtime version **14.50.35719 or later** for Cook. A UE editor license/install and the rights to use the upstream assets are prerequisites, not part of this download.

**Restore local inputs.** This independent source distribution deliberately omits the Blocks maps, plugin materials/images and generated MAVLink headers/XML. Obtain the pinned upstream inputs from a local Microsoft AirSim clone you are entitled to use:

```powershell
git clone --filter=blob:none --no-checkout https://github.com/microsoft/AirSim.git ../AirSim-inputs
& $py -I tools/airsimx/restore_upstream_inputs.py ../AirSim-inputs
```

The restorer reads Git objects at `1ca93f6f77e4e8a39b2b241c1fe2764da4d7dd41` (not the upstream working tree or its latest HEAD), verifies object IDs, and never replaces different files. A partial clone may fetch objects the first time. `--core-only` restores just the protocol inputs for core builds. Restored files are ignored by Git; do not add them to a release. No upstream history is merged into AirsimX.

**Build and stage** into the repository's Blocks project. Run in a clean checkout, with the editor closed. `all` prepares locked dependencies, builds/tests the core, stages the plugin and builds the editor target. It does not launch an aircraft. Use a new output directory for each build record.

```powershell
$ue = Read-Host 'UE 5.8 installation root (the folder containing Engine)'
& $py -I tools/airsimx/build_windows.py all --engine-root "$ue"
if ($LASTEXITCODE -ne 0) { throw 'Build failed. Inspect build/airsimx/build-records before continuing.' }
```

**Migrate the two optical-flow materials locally.** The original shader expressions target UE4. This repository ships the migration source, not modified `.uasset` files. After staging, run the migration on the staged copy before opening or cooking Blocks; repeat after a clean re-stage. Do not modify your upstream clone.

```powershell
$project = (Resolve-Path Unreal/Environments/Blocks/Blocks.uproject).Path
$migrate = (Resolve-Path tools/airsimx/migrate_optical_flow_ue5.py).Path
& "$ue/Engine/Binaries/Win64/UnrealEditor-Cmd.exe" "$project" '-EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities' "-ExecutePythonScript=$migrate" -unattended -nop4 -nosplash -d3d11
```

Check the newest `build/airsimx/asset-migration/*/result.json` for `status: passed` and both materials; process exit alone is insufficient. This is a shader migration, not an optical-flow accuracy measurement. To **open** the scene:

```powershell
& "$ue/Engine/Binaries/Win64/UnrealEditor.exe" "$project" -d3d11
```

Choose the SimpleFlight configuration for a local first run. The [Python client example](../../README.md) reads state without arming. To build a local **Development** package, close the editor and run `& $py -I tools/airsimx/build_windows.py package --engine-root "$ue"`. Output goes to a fresh directory under `build/airsimx/packages/`. A local package is not permission to redistribute its contents.

中文提示：只核对 Python 接口时做到第一节即可。构建 UE 才需要恢复外部输入；工具只读指定的上游提交，不会把旧仓库的 Git 历史合并进来。先看运行记录，再决定是否需要启动仿真。若只想测试软件飞控，从 [PX4 指南](px4-sitl.md) 开始。
