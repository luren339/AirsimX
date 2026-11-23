# Source provenance and third-party notices

AirsimX is an independently maintained **derivative of Microsoft AirSim**, not an official Microsoft product. This repository starts its own Git history; that does not change the provenance of the inherited code. The upstream source baseline is [Microsoft AirSim at 1ca93f6f](https://github.com/microsoft/AirSim/tree/1ca93f6f77e4e8a39b2b241c1fe2764da4d7dd41). Original copyrights and the [MIT license](LICENSE) are retained, together with the [upstream author acknowledgements](AUTHORS.md). Component notices in the source continue to apply. AirSim, Unreal Engine and PX4 names/trademarks belong to their respective owners.

| Component | Distribution in this preview |
| --- | --- |
| AirLib, MavLinkCom wrapper, Unreal plugin sources and client code | Derived from the upstream AirSim code. See the Microsoft MIT license and individual file headers. |
| nlohmann/json | The vendored `AirLib/include/common/common_utils/json.hpp` retains its own MIT copyright and notice. |
| rpclib 2.3.0 and Eigen 3.3.7 | Not bundled. [Dependency pins](tools/airsimx/dependencies.lock.json) identify the archives used by the build. Observe rpclib's MIT and Eigen's per-file licensing, including MPL 2.0 and module-specific terms. |
| Generated MAVLink protocol headers and message definitions | Omitted from this source distribution. The [input restorer](tools/airsimx/restore_upstream_inputs.py) can read the pinned files from a separately obtained upstream tree. Check the relevant generator/header/XML terms for your use. |
| Unreal Engine and Blocks/plugin content | No engine, `.uasset`, `.umap`, cooked package or upstream content images are shipped. Obtain the engine and assets separately under their applicable terms. They are not relicensed by this repository. |
| PX4, ROS 2 and learning libraries | Separately installed; not vendored here. PX4 and ROS packages keep their own licenses. Python dependency versions are in `PythonClient/requirements*.txt`; wheels can include additional third-party notices. |
| External navigation software, bags and training outputs | Not distributed. Only AirsimX-side adapter/recording code is included. |

The README cover combines an original 3D quadrotor render with a procedurally illustrated alpine landscape. It is artwork, not a simulator screenshot or flight record. Star charts use aggregate data from this repository. This inventory describes what is in this source preview; it is not a clearance for redistributing a compiled game, engine content or dependency bundle.
