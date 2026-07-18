# WsProtocol Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Receive `standard_robot_pp_ros2` v2 navigation commands from USART6 and publish only chassis velocity to `HostData`.

**Architecture:** A platform-independent parser validates the 66-byte frame. `WsProtocol` owns `uart_ext_controller`, supplies incoming bytes to it from a LibXR thread, and publishes valid velocity triples to `chassis_data`; `HostData` retains its 150 ms failsafe.

**Tech Stack:** C++20, LibXR UART/Topic/Thread, xrobot YAML, host `assert` tests.

## Global Constraints

- Accept only `0x5A | len=60 | id=0x01 | CRC8 | payload | CRC16` frames.
- Consume only `vx`, `vy`, and `wz`; ignore all other command fields.
- USART6 has a single owner: `WsProtocol`.
- Constants use uppercase names. Do not edit vendor or CubeMX-generated files.

### Task 1: Frame parser

**Files:** Create `Modules/WsProtocol/WsProtocolParser.hpp`; create `tests/ws_protocol_test.cpp`.

**Produces:** `WsProtocolParser::NavigationCommand { float vx; float vy; float wz; }` and `Parser::Push(uint8_t, NavigationCommand&) -> bool`.

- [ ] Write failing tests. Build a 66-byte frame with SOF `0x5A`, length `60`, ID `0x01`, CRC8 over bytes 0-2, CRC16 over bytes 0-63, and IEEE-754 float values at absolute offsets 8, 12, 16. Assert exact mapping for `{1.25F, -2.5F, 0.75F}`. Add cases for delivery split after byte 17, leading noise, bad CRC8, bad CRC16, length 59, and ID 2. Invalid inputs must never publish.
- [ ] Confirm red: the C++20 host compiler command from Task 4 must fail because the header does not exist.
- [ ] Implement `WsProtocolParser.hpp` with `SOF`, `ROBOT_COMMAND_ID`, `HEADER_SIZE=4U`, `PAYLOAD_LENGTH=60U`, `FRAME_SIZE=66U`, `VX_OFFSET=8U`, `VY_OFFSET=12U`, and `WZ_OFFSET=16U`. Use a fixed `std::array<uint8_t, FRAME_SIZE>` and states `WAITING_SOF`, `READING_HEADER`, `READING_BODY`. Validate the header with `LibXR::CRC8::Verify` and the full frame with `LibXR::CRC16::Verify`; do not duplicate CRC algorithms. Reject incorrect length or ID before body reading. Reset on every failure while retaining a current `0x5A` as a new SOF candidate. Decode floats only with `std::memcpy` after CRC16 passes.
- [ ] Confirm green: compile the test with the LibXR `crc_o3.cpp` source and include paths, then run it with no assertion failures.
- [ ] Commit: `git add Modules/WsProtocol/WsProtocolParser.hpp tests/ws_protocol_test.cpp`; `git commit -m "feat: add ws protocol frame parser"`.

### Task 2: LibXR UART module

**Files:** Create `Modules/WsProtocol/WsProtocol.hpp`, `Modules/WsProtocol/CMakeLists.txt`, `tests/ws_protocol_static_regression.ps1`; modify `Modules/modules.yaml`.

**Consumes:** Task 1 parser and `HostData::HostChassisTarget`.

**Produces:** `chassis_data` publications only after full-frame validation.

- [ ] Write failing static checks requiring a module manifest with `uart_name`, `chassis_topic_name`, `task_stack_depth`, `thread_priority`, and dependency `pldx/HostData`; `SetConfig({115200U, LibXR::UART::Parity::NO_PARITY, 8U, 1U})`; `parser_.Push`; and `chassis_topic_.Publish(chassis)`.
- [ ] Confirm red: `pwsh -File tests/ws_protocol_static_regression.ps1` fails because the module header is missing.
- [ ] Implement the constructor: find `LibXR::UART` by `uart_name`; create `LibXR::Topic::CreateTopic<HostData::HostChassisTarget>(chassis_topic_name)`; configure 115200 8N1; create thread `ws_protocol`; register with `app`. Its thread blocks on `uart_->Read({&byte, 1U}, read_op_)`; only on `OK` and a true `parser_.Push` it publishes `HostData::HostChassisTarget{command.vx, command.vy, command.wz}`. `OnMonitor()` is empty, no UART writes occur, and no direct CMD calls occur. Copy the CMake shell from `Modules/HostData/CMakeLists.txt`; add `- pldx/WsProtocol` immediately after `- pldx/HostData`.
- [ ] Confirm green: run the Task 1 compiler command plus `pwsh -File tests/ws_protocol_static_regression.ps1`.
- [ ] Commit: stage only `Modules/WsProtocol`, `Modules/modules.yaml`, and the static test; commit `feat: receive navigation commands over uart6`.

### Task 3: Sentry configuration

**Files:** Modify `User/RobotConfig/sentry.yaml` and `tests/ws_protocol_static_regression.ps1`.

**Consumes:** the Task 2 constructor.

- [ ] Extend the static check to require this block and reject `- chassis_data` inside `sharetopic.topic_configs`:

```yaml
- id: ws_protocol
  name: WsProtocol
  constructor_args:
    uart_name: uart_ext_controller
    chassis_topic_name: chassis_data
    task_stack_depth: 1024
    thread_priority: LibXR::Thread::Priority::MEDIUM
```

- [ ] Confirm red: run `pwsh -File tests/ws_protocol_static_regression.ps1`; it fails because the sentry configuration is unchanged.
- [ ] Insert the block immediately after `hostdata`; remove only `- chassis_data` from `sharetopic.topic_configs`, preserving `sentry_state`, `target_euler`, and `fire_notify` on USB CDC.
- [ ] Confirm green: `pwsh -File tests/ws_protocol_static_regression.ps1`; then `tools/build.sh --skip-format -c User/RobotConfig/sentry.yaml -b build/ws_protocol_sentry`.
- [ ] Commit: stage the YAML and static test; commit `config: route sentry navigation through ws protocol`.

### Task 4: Final evidence

**Files:** Inspect only the Task 1-3 paths.

- [ ] Run `g++ -std=c++20 -Wall -Wextra -Werror -IModules/WsProtocol -IMiddlewares/Third_Party/LibXR/src/utils -IMiddlewares/Third_Party/LibXR/src/core tests/ws_protocol_test.cpp Middlewares/Third_Party/LibXR/src/utils/crc_o3.cpp -o /tmp/ws_protocol_test && /tmp/ws_protocol_test`.
- [ ] Run `pwsh -File tests/ws_protocol_static_regression.ps1`.
- [ ] Run `tools/build.sh --skip-format -c User/RobotConfig/sentry.yaml -b build/ws_protocol_sentry`.
- [ ] Run `git diff --check` and `git status --short`; preserve every unrelated existing change.
