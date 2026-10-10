# agilepf_deploy

## Dependencies

- legged_sdk
- yaml-cpp
- ONNX Runtime (headers and x86_64/arm64 libraries are vendored in `third_party/onnxruntime`)

## Installation

Install the system dependencies above, then configure and build from the repository root:

```sh
cmake -S . -B build
cmake --build build
```

The application loads the sole policy model configured in `params/policy_params.yaml` through ONNX Runtime; it does not use depth images. The build selects the matching ONNX Runtime library and adds its directory to the executable's RPATH, so no separate ONNX Runtime installation or runtime environment setup is needed.

Run with an optional network interface name as the first argument. If omitted, the application uses the local loopback interface (`lo`).

Press Y to enter damping; from damping, press B to begin standing or A to enter policy control directly. A also enters policy control from standing without waiting for the stand-up transition to finish. X disables torque. After a control fault, recovery requires X then Y before re-entering policy control. **Testing configuration:** policy targets are not clamped or checked against the YAML joint limits, so verify motor order, joystick directions, target range, and stand-up pose on a safely suspended robot before enabling torque. The preserved calf start positions are `-1.98`, below the former policy bound of `-1.8`.

Run the hardware-free model and observation checks with `ctest --test-dir build --output-on-failure`.
