# WsProtocol Design

## Goal

Add a `WsProtocol` module that receives the `standard_robot_pp_ros2` v2
command stream from the navigation workstation over `uart_ext_controller`,
validates each frame, and forwards only chassis navigation velocity commands
to the existing `HostData` control path.

## Scope

`WsProtocol` owns reception from `uart_ext_controller` (`USART6`). It accepts
only the workstation's `ID_ROBOT_CMD` frame and publishes its
`speed_vector.vx`, `speed_vector.vy`, and `speed_vector.wz` fields to the
existing `chassis_data` Topic as `HostData::HostChassisTarget`.

The module does not control the gimbal, launcher, chassis pose, leg length,
or `tracking` flag. Those fields are decoded only as necessary to reach the
navigation vector and are otherwise ignored. `SentryProtocol` and the
referee-system command path are outside this change.

## Wire Protocol

The accepted frame is the protocol v2 `SendRobotCmdData` layout used by
`/home/sb/pldx_ws/src/standard_robot_pp_ros2`:

```text
offset  size  field
0       1     SOF = 0x5A
1       1     payload length = 60
2       1     command ID = 0x01 (ID_ROBOT_CMD)
3       1     CRC8 of bytes 0..2
4       4     timestamp (ignored)
8       4     speed_vector.vx, IEEE-754 float
12      4     speed_vector.vy, IEEE-754 float
16      4     speed_vector.wz, IEEE-754 float
20      44    remaining payload (ignored)
64      2     CRC16 of bytes 0..63, little-endian
```

The total frame length is 66 bytes. The header CRC8 and frame CRC16 use the
same LibXR `CRC8` and `CRC16` helpers used by the `Referee` module. Numeric
fields are native little-endian IEEE-754 values, matching both the STM32F407
and the x86 Linux workstation.

## Architecture

`WsProtocol` is a single-purpose UART reader. Its constructor obtains
`LibXR::UART` by name, configures it as 115200 baud, 8 data bits, one stop bit,
and no parity, creates the `chassis_data` Topic with
`HostData::HostChassisTarget`, then starts a medium-priority receive thread.

The thread consumes one byte at a time while synchronizing on `0x5A`.
After a candidate header is complete it verifies CRC8 and requires exactly
`len == 60` and `id == 0x01`. Only then does it read the 62 trailing bytes,
verify CRC16 across the whole frame, copy the three navigation floats into a
`HostData::HostChassisTarget`, and publish it.

Malformed headers, unsupported IDs, incorrect payload lengths, failed CRC8,
failed CRC16, and incomplete reads publish nothing. After any error the parser
continues scanning for a new SOF, allowing recovery from noise, corrupt frames,
and partial reads without resetting the UART.

`HostData` remains the safety boundary: its existing 150 ms freshness timeout
sets the chassis target to zero when valid navigation frames stop arriving.
`WsProtocol` does not introduce a second timeout or directly call `CMD`.

## Configuration

The `sentry.yaml` robot configuration will instantiate `WsProtocol` after
`HostData`, with:

```yaml
- id: ws_protocol
  name: WsProtocol
  constructor_args:
    uart_name: uart_ext_controller
    chassis_topic_name: chassis_data
    task_stack_depth: 1024
    thread_priority: LibXR::Thread::Priority::MEDIUM
```

The existing `SharedTopic` instance remains on `usb_otg_hs_cdc` for unrelated
USB data. Its `chassis_data` registration must be removed so that navigation
control has a single producer and a single input protocol.

## Verification

A host-runnable protocol test will exercise parser behavior without STM32 HAL
or FreeRTOS dependencies. It must cover a valid frame with exact `vx`, `vy`,
and `wz` output; split-frame delivery; leading noise; invalid CRC8; invalid
CRC16; incorrect payload length; unsupported command ID; and valid frames
whose ignored fields contain non-zero data. The target build must compile the
updated `sentry.yaml` using the repository's `tools/build.sh --skip-format`
pipeline.

## Constraints

- Use C++20 and compile cleanly under the project's global `-Werror` flags.
- Follow the project's naming convention, including uppercase `const` and
  `constexpr` identifiers.
- Do not modify vendor code under `Drivers/` or `Middlewares/`.
- Do not edit CubeMX-generated `Core/` code for this feature.
- Keep `uart_ext_controller` exclusively owned by `WsProtocol` in the sentry
  configuration.
