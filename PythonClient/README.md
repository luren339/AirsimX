# Python API for AirsimX

This maintained client derives from [Microsoft AirSim](https://github.com/microsoft/AirSim) and keeps the `airsim` import name, public vehicle methods, NED coordinate conventions, and MessagePack-RPC wire protocol. AirsimX is a community fork, not a Microsoft release.

## Installation

Package metadata requires Python 3.10 or newer. The current Windows validation environment uses Python 3.12.10; other interpreter/platform combinations are not implied to have passed the same runtime suite.

From the checkout root, using a virtual environment:

```bash
python -m pip install -r PythonClient/requirements-v01-windows.txt
```

```bash
python -m pip install --no-deps ./PythonClient
```

Dependencies are current `msgpack`, NumPy and OpenCV packages. The client no longer uses `msgpack-rpc-python` or Tornado 4.5.3; do not install the old RPC dependency or use administrator/sudo installation instructions from upstream for this version. The pinned Windows requirements also include optional `psutil` diagnostics for the runtime acceptance runner.

## Basic use

Start a configured AirSim/AirsimX simulator before connecting:

```python
import airsim

with airsim.MultirotorClient(timeout_value=30) as client:
    print(client.ping())
    print(client.getMultirotorState())
```

Car driving is not part of the validated UE5 single-multirotor preview scope.

## Transport compatibility

- Asynchronous methods still return objects with `.get()` and `.join()`. Use `.get()` to propagate remote failures. Legacy `.join()` waits for completion without raising the stored RPC failure.
- `VehicleClient.close()` and the context manager close sockets and fail outstanding requests. The internal `client.client.close()` entry point remains available.
- RPC timeouts use a finite positive wall-clock budget. Timing out or closing the connection does **not** cancel work already running in the simulator; use the appropriate control API and safe landing logic. In-flight requests are never automatically replayed after a disconnect.
- Exceptions are `airsim.rpc.RPCError`, `RPCTimeoutError` and `RPCTransportError`. The latter two also derive from the corresponding built-in timeout/connection error types. Third-party `msgpackrpc` internal classes, exception identity and Tornado event-loop integration are not retained.
- Binary helpers support bytes and legacy MessagePack string data and return writable NumPy arrays.

## Tests

```bash
python -m unittest discover -s PythonClient/tests -v
```

Loopback protocol tests are not flight verification. The repository's `tools/airsimx/validate_runtime.py` performs the bounded UE runtime acceptance profile; see [Windows development instructions](../docs/airsimx/windows-development.md).

Original API documentation remains available at <https://microsoft.github.io/AirSim/apis/>. Original Microsoft attribution and the MIT license are retained in `LICENSE`.
