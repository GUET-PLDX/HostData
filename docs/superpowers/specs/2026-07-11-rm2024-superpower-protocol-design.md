# RM2024 SuperPower Protocol Compatibility Design

**Goal:** Make the robot-controller `SuperPower` module communicate exactly
with the RM2024 SuperCapacitorController firmware in
`.Data/RM2024-SuperCapacitorController-master`.

**Scope:** This change affects only the controller-side SuperPower CAN driver
and its documentation. Existing robot YAML configuration, CAN hardware
mapping, `PowerControl`, and chassis modules remain unchanged.

## Protocol Contract

All frames use classic CAN, standard identifiers, DLC 8, and STM32 little
endian byte order.

| Direction | CAN ID | Payload |
|---|---:|---|
| Supercap controller to robot controller | `0x051` | `error_code`, `float chassis_power`, `uint16_t chassis_power_limit`, `uint8_t cap_energy` |
| Robot controller to supercap controller | `0x061` | control flags, `uint16_t referee_power_limit`, `uint16_t referee_energy_buffer`, three reserved bytes |

The payload definitions are taken directly from
`Core/Src/Communication.cpp` in the reference firmware.

### Feedback Frame: `0x051`

```cpp
struct __attribute__((packed)) FeedbackData {
  uint8_t error_code;
  float chassis_power;
  uint16_t chassis_power_limit;
  uint8_t cap_energy;
};
```

- `error_code` is the reference firmware's status byte. Bits `0..6` are
  fault flags; bit `7` is set when the power output is disabled.
- `chassis_power` is an IEEE-754 floating-point watt value. It is not an
  offset-binary encoded integer.
- `chassis_power_limit` is the controller-computed maximum chassis power in
  watts.
- `cap_energy` is the normalized capacitor energy fraction multiplied by
  `255`.

`GetChassisPower()` returns `chassis_power`. `GetCapEnergy()` returns
`cap_energy / 255.0f`, preserving all current `PowerControl` and chassis
BOOST/UI thresholds. New read-only accessors may expose the status byte and
reported power limit, but no consumer may treat `cap_energy` as output
capability.

### Command Frame: `0x061`

```cpp
struct __attribute__((packed)) CommandData {
  uint8_t flags;
  uint16_t referee_power_limit;
  uint16_t referee_energy_buffer;
  uint8_t reserved[3];
};
```

- `flags & 0x01` is always set to enable DCDC output.
- `flags & 0x02` remains clear; the robot controller never requests a
  supercap-controller reset.
- Bits `2..7` remain clear.
- `referee_power_limit` comes from
  `Referee::ChassisPack::rs.chassis_power_limit`.
- `referee_energy_buffer` comes from
  `Referee::ChassisPack::power_buffer`.
- The final three bytes are zero.

The driver must use packed structs, `static_assert(sizeof(...) == 8)`, and
`LibXR::Memory::FastCopy` to prevent compiler layout assumptions from
changing the wire format.

## Runtime Design

The CAN receive callback validates DLC and stores only the latest feedback
frame plus its receive timestamp. Parsing and control-frame transmission run
from a periodic LibXR timer task, not from the CAN ISR.

The timer sends `0x061` every 5 ms after the module is constructed. It uses
the latest `chassis_ref` data. A missing or stale referee topic publishes a
safe zero power limit and zero energy buffer rather than continuing to send
obsolete limits.

The module is online after its first valid feedback frame and offline once no
valid `0x051` frame has arrived for 100 ms. On transition offline, it clears
all externally visible feedback values. Identical feedback frames remain
valid: the reference firmware may legitimately transmit unchanged values
while the robot is stationary, so no repeated-frame offline rule is allowed.

## Compatibility and Safety

- Support only the confirmed RM2024 protocol. Do not retain the unrelated
  offset-binary/new-protocol parsing path and do not add YAML protocol
  selection.
- Do not send CAN messages from an ISR.
- Keep the constructor contract unchanged: `can_bus_name` remains its only
  YAML argument.
- Keep the dependency on `Referee` because command construction consumes the
  existing `chassis_ref` topic.
- Protect foreground reads and callback/timer writes using the existing LibXR
  queue or synchronization pattern selected during implementation.

## Verification

1. Compile-time checks prove both CAN payloads are exactly eight bytes.
2. Protocol tests use fixed byte vectors to verify little-endian float parsing,
   energy normalization, status retention, and exact `0x061` byte layout.
3. Builds of `User/RobotConfig/sentry_chassis.yaml` and
   `User/RobotConfig/sentry_gimbal.yaml` succeed with `-Werror`.
4. Hardware CAN capture verifies a periodic `0x061` stream contains the live
   referee power limit and buffer energy, while `0x051` produces the expected
   measured-power and energy values.
5. Hardware disconnect and reconnect verifies offline at 100 ms, state
   clearing, and recovery after the next valid feedback frame.

## Out of Scope

- Changing the RM2024 supercap-controller firmware.
- Supporting the former `0x052` or current incompatible feedback layouts.
- Retuning chassis power allocation or BOOST thresholds.
- Changing CubeMX-generated files, vendor drivers, or LibXR.
