# WsProtocol Refactor Design

## Status

This design supersedes `2026-07-11-ws-protocol-design.md` and its associated
implementation plan. The older documents remain as historical records of the
initial receive-only parser architecture.

## Goal

Refactor `Modules/WsProtocol` into a single-file XRobot module whose structure
and extension workflow match `Modules/Referee`, while preserving the existing
`pldx_ws` version 2 wire protocol.

The module must:

- receive the complete PC-to-MCU `ID_ROBOT_CMD` payload;
- publish its chassis velocity through the existing `HostData` path;
- continuously publish zero chassis velocity when navigation commands become
  stale;
- support all MCU-to-PC command types currently defined by `pldx_ws`;
- make adding a command a small enum, payload, switch case, and wrapper change;
- remain the only owner of `uart_ext_controller` on the sentry gimbal board.

## Non-Goals

- Do not modify the `pldx_ws` wire protocol or source tree.
- Do not make `WsProtocol` aware of CMD mode or chassis mode.
- Do not force the chassis into `RELAX` when the workstation link is stale.
- Do not publish the received gimbal, launcher, pose, or tracking fields yet.
- Do not add semantic validation for finite values or command ranges.
- Do not add a generic handler registry, a separate codec class, or another
  protocol source file.

## Design Principles

1. `WsProtocol.hpp` is the only module entry and the only production protocol
   header. Auxiliary files are justified only by independent reuse or
   independent evolution; the current protocol does not require them.
2. Follow `Referee`'s linear organization: protocol types, cached data, public
   send API, receive loop, parsing, publication, then state members.
3. Keep transport framing separate from command meaning. Framing validates
   SOF, length, CRC8, and CRC16; the command switch copies known payloads.
4. Preserve the existing wire layout exactly while using project-compliant,
   clear C++ identifiers.
5. Treat link activity and chassis-command freshness as different facts.
6. Prefer a deterministic fixed-memory design. No allocation occurs in the
   receive or transmit path.

## Module Boundary

`Modules/WsProtocol/WsProtocol.hpp` defines one
`WsProtocol : public LibXR::Application` class. `WsProtocolParser.hpp` is
removed.

The public protocol surface contains:

- `Status`;
- `RxCommandID`;
- `TxCommandID`;
- `Header`;
- the full robot-command receive payload;
- all MCU-to-PC business data types defined by `pldx_ws`;
- the aggregate `Data` type;
- raw and typed `SendFrame()` overloads;
- one named send method for every `TxCommandID`.

The `Data` type is public so consumers can name the protocol model, matching
`Referee`. The `data_` instance is private and has no getter or mutable access.

The constructor contract is:

```cpp
WsProtocol(LibXR::HardwareContainer& hw,
           LibXR::ApplicationManager& app,
           uint32_t task_stack_depth_uart,
           const char* uart,
           uint32_t baudrate,
           const char* chassis_topic_name,
           LibXR::Thread::Priority thread_priority_uart);
```

For deliberate parity with the current `Referee` implementation:

- `app` is unused and the module does not call `app.Register(*this)`;
- `thread_priority_uart` remains part of the generated constructor contract,
  but the receive thread is created at `LibXR::Thread::Priority::MEDIUM`;
- `OnMonitor()` is empty;
- the receive loop has no additional sleep.

The module also registers one 1 ms check task with LibXR's global `Timer`.
This is not a second module-owned thread: LibXR runs all timer tasks from its
shared timer thread. The task evaluates the 50 ms command deadline on each
tick, while stale zero publications remain limited to one every 50 ms. The
short check interval is required because a 50 ms periodic check can miss a
just-arrived command by one phase and delay first zero output until almost
100 ms after the last valid command.

## Wire Protocol

Every frame uses the existing `pldx_ws` version 2 layout:

```text
offset  size  field
0       1     SOF = 0x5A
1       1     payload length, 0..255
2       1     direction-specific command ID
3       1     CRC8 over bytes 0..2
4       len   payload
4+len   2     CRC16 over header and payload, little-endian
```

The maximum frame size is `4 + 255 + 2 = 261` bytes. RX and TX buffers both
cover this full size.

`RxCommandID` and `TxCommandID` are separate because ID values are reused by
direction. In particular, `0x01` means PC-to-MCU `ROBOT_COMMAND` on receive and
MCU-to-PC `DEBUG` on transmit.

Wire structures use `[[gnu::packed]]`. Bit fields mirror the existing
`Referee` style and current `pldx_ws` layout. Single-byte Boolean wire fields,
including `tracking`, use `uint8_t` and are converted at the business boundary.
Names such as `leg_lenth` are corrected to project-compliant names such as
`leg_length` without changing field order or width.

No layout `static_assert` is added. Compatibility is maintained by keeping the
two implementations synchronized and validating communication during system
integration.

## Receive Model

The receive thread and global timer task have separate high-level loops:

```text
RX thread:
  FindHeader()
  ParseData()
  if parse succeeded: Publish()

Global Timer task every 1 ms:
  CheckChassisCommandFreshness()
```

### Header Search

`FindHeader()` reads one byte at a time until it sees `0x5A`.

- A 50 ms timeout while waiting for a byte sets `data_.status` to `OFFLINE`.
- After SOF, the remaining three header bytes are read in one operation.
- Matching the chosen `Referee` behavior, the return value of that three-byte
  read is not checked before CRC8 verification.
- A header passing CRC8 immediately sets `data_.status` to `RUNNING` and ends
  the search.

The internal status therefore describes valid-header activity, not complete
frame validity and not chassis-command freshness. It is not published.

### Payload Parsing

`ParseData()` reads `header.len + 2` bytes into the fixed receive buffer and
verifies CRC16 over the complete frame. It then switches on `RxCommandID`.

The initial receive command set contains `ROBOT_COMMAND = 0x01`. Its payload
stores all 60 existing bytes:

- workstation timestamp;
- `speed_vector.vx`, `vy`, and `wz`;
- chassis roll, pitch, yaw, and leg length;
- gimbal control flag, position, velocity, and acceleration;
- launcher fire and friction-wheel flags;
- tracking flag.

A known command is accepted when `len >= sizeof(payload)`. Any extra tail is
ignored, matching `Referee`'s forward-compatible copy rule. A shorter payload,
failed read, or failed CRC16 returns `false` without changing that command's
cache.

An unknown but CRC-valid ID leaves the link `RUNNING`, returns `false`, does
not update cached command data, and does not call normal publication.

CRC-valid payload values are copied without checking ranges, `NaN`, or
infinity. Semantic validation remains outside this communication module.

Adding a receive command requires:

1. one `RxCommandID` value;
2. one packed payload type and one `Data` field;
3. one `ParseData()` switch case;
4. any required additions to `Publish()`.

## Publication

`Publish()` follows the aggregate-cache style used by `Referee`. Any known
command that parses successfully may cause current cached outputs to be
rebuilt.

The only current output is a non-retained
`HostData::HostChassisTarget` Topic named by `chassis_topic_name`:

```text
RobotCommand.speed_vector.vx -> HostChassisTarget.vx
RobotCommand.speed_vector.vy -> HostChassisTarget.vy
RobotCommand.speed_vector.wz -> HostChassisTarget.w
```

The remaining robot-command fields are cached but not published. The Topic
does not retain its last value because it carries a freshness-sensitive
control command, not persistent state.

## Chassis Freshness and Zero Output

Link status cannot provide the safety guarantee. Other valid commands, valid
headers, or continuous noise may keep the UART active without providing a new
navigation command. The module therefore tracks the time of the last complete,
CRC-valid, successfully parsed `ROBOT_COMMAND` separately.

The rules are:

- only a successfully parsed `ROBOT_COMMAND` refreshes chassis freshness;
- the command expires 50 ms after that timestamp;
- if no valid command has ever arrived, it first expires 50 ms after module
  startup;
- the 1 ms Timer task observes expiry on the first scheduler tick at or after
  the 50 ms deadline, rather than waiting for the phase of a 50 ms check;
- while expired, the module publishes a zero `HostChassisTarget` immediately
  on that first expired check and every 50 ms thereafter;
- the periodic LibXR Timer task performs expiry and zero publication
  independently of the blocking UART receive thread, so noise, partial frames,
  and unrelated commands cannot delay zero output;
- if `Publish()` is triggered by another command while the robot command is
  stale, it publishes zero rather than the cached old velocity;
- receiving a new valid robot command immediately resumes normal publication.

The cached full payload is not erased when stale. Only the control output is
forced to zero.

One mutex protects the startup timestamp, last-valid-command timestamp,
last-zero-publication timestamp, and chassis publication. The timer holds it
while publishing a stale zero, and the receive path uses the same mutex while
marking a command fresh and publishing its velocity. This orders recovery
against a concurrent timer callback so that a late stale-zero publication
cannot overwrite a newly recovered command.

`WsProtocol` does not inspect navigation mode. `HostData` feeds these values
into CMD's AI source. Operator mode ignores the AI source; automatic mode uses
the continuously refreshed zero command while the link is stale. The module
does not change chassis mode, so modes such as `FOLLOW` or `ROTOR` retain their
own behavior.

This gimbal-side guard is necessary because the gimbal `DualBoard` otherwise
continues sending its cached `chassis_cmd` every 10 ms. Continued CAN traffic
would prevent the chassis board's 100 ms dual-board offline guard from firing.

## Transmit Model

`TxCommandID` covers every MCU-to-PC command currently defined by
`pldx_ws`:

| Value | Command |
| --- | --- |
| `0x01` | `DEBUG` |
| `0x02` | `IMU` |
| `0x03` | `ROBOT_STATE_INFO` |
| `0x04` | `EVENT_DATA` |
| `0x05` | `PID_DEBUG` |
| `0x06` | `ALL_ROBOT_HP` |
| `0x07` | `GAME_STATUS` |
| `0x08` | `ROBOT_MOTION` |
| `0x09` | `GROUND_ROBOT_POSITION` |
| `0x0A` | `RFID_STATUS` |
| `0x0B` | `ROBOT_STATUS` |
| `0x0C` | `JOINT_STATE` |
| `0x0D` | `BUFF` |
| `0x0E` | `GIMBAL_STATE` |

The raw public API mirrors `Referee`:

```cpp
LibXR::ErrorCode SendFrame(TxCommandID command_id, const void* payload,
                           uint16_t payload_len);

template <typename PayloadType>
LibXR::ErrorCode SendFrame(TxCommandID command_id,
                           const PayloadType& payload);
```

Each command also has a named public `Send*()` method. Named methods accept
only the business data substructure. They read `LibXR::Timebase`, add the
32-bit timestamp, and pass the complete payload to `SendFrame()`.

`SendFrame()`:

1. locks one TX mutex for the complete operation;
2. rejects payloads above 255 bytes or invalid pointer/length combinations;
3. writes SOF, length, command ID, and CRC8;
4. copies the payload;
5. appends little-endian CRC16;
6. performs one UART write using a 5000 ms `WriteOperation`;
7. returns the resulting `LibXR::ErrorCode` unchanged.

The module does not retry, queue, schedule, or rate-limit transmissions.
Callers own transmission frequency. The mutex only prevents concurrent callers
from interleaving frames in the shared TX buffer.

### UART Transmit Capacity

The raw API's 255-byte payload produces a 261-byte frame. LibXR's STM32 UART
driver splits the configured TX backing storage into two equal DMA blocks and
uses one half as the write-port byte queue. The existing USART6 backing size of
512 bytes therefore exposes only 256 bytes and would return
`LibXR::ErrorCode::FULL` for a maximum frame.

`User/libxr_config.yaml` and the generated `User/app_main.cpp` hardware map
increase the USART6 TX backing storage to 528 bytes. This is the smallest valid
LibXR double-buffer size that provides at least 261 bytes per half while
remaining divisible by `2 * alignof(size_t)` on the target; each half is 264
bytes. The generated hardware-map edit is committed separately from functional
configuration changes.

Adding a transmit command requires:

1. one `TxCommandID` value;
2. one packed business data type;
3. one timestamped payload type;
4. one thin named send wrapper.

## Double-Board Configuration

The production path is the double-board sentry configuration:

```text
pldx_ws
  -> uart_ext_controller
  -> sentry gimbal WsProtocol
  -> chassis_data
  -> HostData
  -> CMD / chassis_cmd
  -> gimbal DualBoard CAN control frame
  -> chassis DualBoard / chassis_cmd
  -> Chassis
```

`User/RobotConfig/sentry_gimbal.yaml` changes as follows:

- instantiate `WsProtocol` immediately after `HostData`;
- configure `task_stack_depth_uart`, `uart`, `baudrate`,
  `chassis_topic_name`, and `thread_priority_uart`;
- remove `chassis_data` from the local `SharedTopic` receive list so there is
  one producer.

`User/RobotConfig/sentry_chassis.yaml` does not instantiate `WsProtocol`.

The unused single-board `User/RobotConfig/sentry.yaml` remains internally
valid:

- remove its `WsProtocol` instance;
- restore `chassis_data` to its `SharedTopic` receive list.

Generated `User/xrobot_main.hpp` is never edited manually.

## Files

Implementation changes are limited to:

- modify `Modules/WsProtocol/WsProtocol.hpp`;
- modify `Modules/WsProtocol/README.md`;
- delete `Modules/WsProtocol/WsProtocolParser.hpp`;
- modify `User/libxr_config.yaml`;
- modify generated `User/app_main.cpp` in a separate commit;
- modify `User/RobotConfig/sentry_gimbal.yaml`;
- modify `User/RobotConfig/sentry.yaml`;
- delete `tests/ws_protocol_test.cpp`;
- delete `tests/ws_protocol_static_regression.ps1`.

`Modules/WsProtocol/CMakeLists.txt`, vendor code, CubeMX code, LibXR, and
`pldx_ws` remain unchanged unless compilation proves a directly related build
registration change is necessary.

## Verification

There is no standalone host parser test and no PowerShell static regression
test after this refactor. Automated evidence is:

```bash
tools/format_code.sh --check
tools/build.sh --skip-format \
  -c User/RobotConfig/sentry_gimbal.yaml \
  -b build/ws_protocol_sentry_gimbal
git diff --check
```

Hardware integration must confirm:

- the complete robot-command payload matches the existing `pldx_ws` layout;
- valid commands reach the chassis through the double-board path;
- 50 ms without a valid robot command produces continuous zero velocity;
- other traffic and corrupt frames do not preserve stale velocity;
- valid navigation resumes immediately after link recovery;
- every named MCU-to-PC send method produces the ID, length, CRC, timestamp,
  and payload layout defined by the unchanged `pldx_ws`; implemented
  `pldx_ws` handlers decode their fields, while the currently stubbed
  `PID_DEBUG` handler is validated by raw serial capture;
- a raw 255-byte payload is transmitted as one 261-byte frame without
  `LibXR::ErrorCode::FULL`.

## Accepted Trade-Offs

- The single header will be longer, but its control flow matches the project's
  dominant module style and keeps extension localized.
- The 1 ms freshness-check task allocates one LibXR Timer control block during
  initialization and then reuses the framework's global timer thread. It
  publishes only at the approved 50 ms stale-output cadence.
- The remaining-header UART read deliberately mirrors `Referee` and ignores
  its return code before CRC8 verification.
- The thread-priority constructor argument deliberately remains unused while
  the receive thread is fixed at medium priority.
- Wire layout is not protected by compile-time size or offset assertions.
- Protocol behavior is verified by target compilation and hardware
  integration rather than a host unit test.
