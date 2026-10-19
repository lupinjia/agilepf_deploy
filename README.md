# agilepf_deploy

Deployment framework for motion control policies on AgilePF (Agile Point-Foot biped robot). This framework utilizes OnnxRuntime to infer the policy and FastDDS to communicate with hardware codes.

## Dependencies

- [legged_sdk](https://github.com/lupinjia/legged_sdk)
- yaml-cpp (`sudo apt install libyaml-cpp-dev`)
- ONNX Runtime (headers and x86_64/arm64 libraries are vendored in `third_party/onnxruntime`)

## Usage

Install the system dependencies above, then configure and build from the repository root:

```sh
cmake -S . -B build
cmake --build build
```

The application loads the sole policy model configured in `params/policy_params.yaml` through ONNX Runtime. The build selects the matching ONNX Runtime library and adds its directory to the executable's RPATH, so no separate ONNX Runtime installation or runtime environment setup is needed.

Run the application with:

```sh
./agilepf_deploy
```

Run with an optional network interface name as the first argument to pass the data through ethernet interface. If omitted, the application uses the local loopback interface (`lo`) and `domain ID = 1`:

```sh
./agilepf_deploy network_interface_name
```

### Joystick Control

The correspondence between joystick buttons and robot states is as follows:

| Joystick Button | Robot State |
| ---------------- | ------------ |
| Y               | damping (can be entered from any state) |
| B               | standing (can only be entered from damping state) |
| A               | policy control (can be entered from damping or standing state) |
| X               | zero torque (can be entered from any state) |

> [!warning] If the robot shows any abnormal behavior, please enter damping or zero torque to avoid further damage.

### Adjust 

### Data Log

We have two log files:
- `message_log.txt`: Log messages from the application. It includes time between successive low state subscription and low command publication. Average time consumed for control calculation is also logged.
  
  To enable it, use `cmake -S . -B build -DPRINT_DEBUG=ON` when compiling. It will print debug messages to the console, and also log them to `message_log.txt`.
  
- `robot_state_log.txt`: Log messages for the robot state (50Hz). It incorporates compute_time, state (0: damping, 1: stand, 2: ctrl, 3: test, 4: zero torque), cmd, projected_gravity, gyro, joint_pos, joint_vel and actions. 

### Use Realtime Scheduling

To enable realtime scheduling, you need to [install PREEMPT_RT patch for linux kernel](https://realtime-linux.org/getting-started-with-preempt_rt-guide/) first.

Then, you need to run the application with `sudo` privilege:

```sh
sudo -i
./agilepf_deploy network_interface_name
```