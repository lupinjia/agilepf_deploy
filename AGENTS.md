# Project notes

- Configure/build from the repository root: `cmake -S . -B build && cmake --build build`.
- Run from `build/`: the application resolves `../params` relative to the working directory. Logs are created under the working directory's `logs/<timestamp>/`.
- `PRINT_DEBUG` is a CMake option (default `OFF`); enable with `cmake -S . -B build -DPRINT_DEBUG=ON` and rebuild. Its additional diagnostics go to `message_log.txt`, not the console. See `include/robot_controller.hpp` for the three conditional logging blocks.
- Runtime CSV data is written to `robot_state_log.csv` by `RobotController::writeLog()` each successful control step. `action_*` contains raw policy outputs, not desired joint positions. The `log.csv` path in `src/main.cpp` is unused and does not create a file.
