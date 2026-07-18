# RM2024 SuperPower Protocol Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `SuperPower` communicate exactly with the confirmed RM2024 SuperCapacitorController firmware.

**Architecture:** Put the two eight-byte CAN payloads in a standard-C++ codec header that is host-testable. Make `SuperPower.hpp` a LibXR adapter: callbacks enqueue frames, a 5 ms timer parses the latest data, refreshes online state, and sends commands outside ISR context.

**Tech Stack:** C++17, LibXR CAN/Topic/Timer/LockFreeQueue, STM32 classic CAN, PowerShell, `tools/build.sh`.

## Global Constraints

- Target is STM32F407, C++17, `-Werror`, `-fno-rtti`, and `-fno-exceptions`.
- Do not modify `Core/Src` outside USER CODE blocks, `Drivers/`, or `Middlewares/`.
- Preserve the YAML constructor contract: only `can_bus_name`.
- Use packed structs, `static_assert(sizeof(...) == 8)`, and `LibXR::Memory::FastCopy` in target code.
- Support only RM2024 `0x051/0x061`; remove all `0x052` and offset-binary paths.

---

### Task 1: Add a Host-Testable Protocol Codec

**Files:**
- Create: `Modules/SuperPower/SuperPowerProtocol.hpp`
- Create: `tests/superpower_rm2024_protocol_test.cpp`

**Interfaces:**
- Produces: `SuperPowerProtocol::FeedbackData`, `CommandData`, `DecodeFeedback`, and `EncodeCommand`.
- Consumed by: `SuperPower.hpp` and the host test.

- [ ] **Step 1: Write the failing byte-vector test**

Create `tests/superpower_rm2024_protocol_test.cpp`:

```cpp
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include "SuperPowerProtocol.hpp"
int main() {
  constexpr std::array<uint8_t, 8> RX = {0x83, 0x00, 0x00, 0x2A,
                                          0x42, 0x96, 0x00, 0x80};
  const auto feedback = SuperPowerProtocol::DecodeFeedback(RX.data());
  assert(feedback.error_code == 0x83U);
  assert(std::fabs(feedback.chassis_power - 42.5f) < 0.0001f);
  assert(feedback.chassis_power_limit == 150U);
  assert(feedback.cap_energy == 128U);
  SuperPowerProtocol::CommandData command{};
  command.flags = SuperPowerProtocol::ENABLE_DCDC_MASK;
  command.referee_power_limit = 120U;
  command.referee_energy_buffer = 55U;
  constexpr std::array<uint8_t, 8> EXPECTED = {0x01, 0x78, 0x00, 0x37,
                                                0x00, 0x00, 0x00, 0x00};
  assert(SuperPowerProtocol::EncodeCommand(command) == EXPECTED);
}
```

- [ ] **Step 2: Confirm the test fails before the header exists**

Run `g++ -std=c++17 -Wall -Wextra -Werror -IModules/SuperPower tests/superpower_rm2024_protocol_test.cpp -o /tmp/superpower_rm2024_protocol_test`.

Expected: compiler error for missing `SuperPowerProtocol.hpp`.

- [ ] **Step 3: Implement the codec**

Create `Modules/SuperPower/SuperPowerProtocol.hpp`:

```cpp
#pragma once
#include <array>
#include <cstdint>
#include <cstring>
namespace SuperPowerProtocol {
constexpr uint32_t FEEDBACK_ID = 0x051U;
constexpr uint32_t COMMAND_ID = 0x061U;
constexpr uint8_t ENABLE_DCDC_MASK = 0x01U;
constexpr uint8_t SYSTEM_RESTART_MASK = 0x02U;
struct __attribute__((packed)) FeedbackData {
  uint8_t error_code;
  float chassis_power;
  uint16_t chassis_power_limit;
  uint8_t cap_energy;
};
struct __attribute__((packed)) CommandData {
  uint8_t flags;
  uint16_t referee_power_limit;
  uint16_t referee_energy_buffer;
  uint8_t reserved[3];
};
static_assert(sizeof(FeedbackData) == 8U, "FeedbackData must be eight bytes");
static_assert(sizeof(CommandData) == 8U, "CommandData must be eight bytes");
inline FeedbackData DecodeFeedback(const uint8_t* data) {
  FeedbackData feedback{};
  std::memcpy(&feedback, data, sizeof(feedback));
  return feedback;
}
inline std::array<uint8_t, sizeof(CommandData)> EncodeCommand(const CommandData& command) {
  std::array<uint8_t, sizeof(CommandData)> bytes{};
  std::memcpy(bytes.data(), &command, sizeof(command));
  return bytes;
}
}  // namespace SuperPowerProtocol
```

- [ ] **Step 4: Run the test and commit**

Run `g++ -std=c++17 -Wall -Wextra -Werror -IModules/SuperPower tests/superpower_rm2024_protocol_test.cpp -o /tmp/superpower_rm2024_protocol_test` followed by `/tmp/superpower_rm2024_protocol_test`.

Expected: both commands exit `0`.

Commit only the two new files with `git commit -m "test: cover RM2024 superpower protocol bytes"`.

### Task 2: Refactor the LibXR CAN Adapter

**Files:**
- Modify: `Modules/SuperPower/SuperPower.hpp`

**Interfaces:**
- Consumes: the Task 1 protocol header.
- Produces: unchanged `GetChassisPower()`, normalized `GetCapEnergy()`, and `IsOnline()`; add `GetErrorCode()` and `GetChassisPowerLimit()`.

- [ ] **Step 1: Replace frame declarations and imports**

Remove the local IDs, offset decoder, `StatusData`, old `CommandData`, repeated-frame counter, and output-capability fields. Include `SuperPowerProtocol.hpp`, `lockfree_queue.hpp`, and `timer.hpp`.

Add exactly these transport records and constants:

```cpp
static constexpr uint32_t COMMAND_PERIOD_MS = 5U;
static constexpr uint32_t STATUS_RX_TIMEOUT_MS = 100U;
static constexpr uint32_t REFEREE_RX_TIMEOUT_MS = 1000U;
struct FeedbackFrame { LibXR::CAN::ClassicPack pack; uint32_t received_time_ms; };
struct RefereeData { uint16_t power_limit; uint16_t energy_buffer; uint32_t received_time_ms; };
```

- [ ] **Step 2: Move callback work to single-slot queues**

The CAN callback must reject `dlc < sizeof(SuperPowerProtocol::FeedbackData)`, timestamp the valid `ClassicPack`, and push it to `feedback_queue_{1}`. If full, pop once and retry. It must not decode or send CAN.

The `chassis_ref` callback must queue this exact data, replacing older data when full:

```cpp
RefereeData data{chassis_pack.rs.chassis_power_limit,
                 chassis_pack.power_buffer,
                 static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds())};
```

- [ ] **Step 3: Add the timer update flow**

Create and start a LibXR timer in the constructor:

```cpp
timer_handle_ = LibXR::Timer::CreateTask(TimerTask, this, COMMAND_PERIOD_MS);
LibXR::Timer::Add(timer_handle_);
LibXR::Timer::Start(timer_handle_);
```

`TimerTask` calls `Update()`. `Update()` drains both queues, decodes feedback with `DecodeFeedback`, caches `error_code`, `chassis_power`, `chassis_power_limit`, `cap_energy`, referee power limit, and referee buffer, then calls `RefreshOnlineState(now_ms)` and `SendCommandFrame(now_ms)`.

- [ ] **Step 4: Implement state, timeout, and command bytes**

Protect public state with `LibXR::Mutex`. After the first valid feedback, online remains true until `now_ms - last_feedback_rx_time_ms_ > 100U`; clear all feedback values on timeout. Never use repeated-frame content to decide online state.

`GetCapEnergy()` must return:

```cpp
return static_cast<float>(cap_energy_) / 255.0f;
```

Every 5 ms, create the following command. Track referee-data presence with an explicit `bool referee_received_`; values remain zero when it is false or the data is older than 1000 ms. Do not use a zero timestamp as the absence sentinel because a valid first update can occur at tick zero.

```cpp
SuperPowerProtocol::CommandData command{};
command.flags = SuperPowerProtocol::ENABLE_DCDC_MASK;
if (last_referee_rx_time_ms_ != 0U &&
    now_ms - last_referee_rx_time_ms_ <= REFEREE_RX_TIMEOUT_MS) {
  command.referee_power_limit = referee_power_limit_;
  command.referee_energy_buffer = referee_energy_buffer_;
}
LibXR::CAN::ClassicPack tx_pack{};
tx_pack.id = SuperPowerProtocol::COMMAND_ID;
tx_pack.type = LibXR::CAN::Type::STANDARD;
tx_pack.dlc = sizeof(command);
LibXR::Memory::FastCopy(tx_pack.data, &command, sizeof(command));
can_->AddMessage(tx_pack);
```

- [ ] **Step 5: Format, build, and commit**

Run `tools/format_code.sh`, then `tools/build.sh --skip-format -c User/RobotConfig/sentry_chassis.yaml -b build/sentry_chassis` and `tools/build.sh --skip-format -c User/RobotConfig/sentry_gimbal.yaml -b build/sentry_gimbal`.

Expected: both succeed with no warnings or errors.

Commit only `Modules/SuperPower/SuperPower.hpp` with `git commit -m "feat: support RM2024 superpower CAN protocol"`.

### Task 3: Lock the Contract with Documentation and Regression Coverage

**Files:**
- Modify: `Modules/SuperPower/README.md`
- Create: `tests/superpower_rm2024_protocol_static_regression.ps1`

**Interfaces:**
- Documents the RM2024-only contract and checks that future edits retain it.

- [ ] **Step 1: Add a PowerShell static regression script**

Create `tests/superpower_rm2024_protocol_static_regression.ps1`:

```powershell
$ErrorActionPreference = 'Stop'
function Assert-Contains {
  param([string]$Path, [string]$Pattern, [string]$Message)
  if (-not (Select-String -Path $Path -Pattern $Pattern -Quiet -CaseSensitive)) {
    throw $Message
  }
}
function Assert-NotContains {
  param([string]$Path, [string]$Pattern, [string]$Message)
  if (Select-String -Path $Path -Pattern $Pattern -Quiet -CaseSensitive) {
    throw $Message
  }
}
$protocol = 'Modules/SuperPower/SuperPowerProtocol.hpp'
$module = 'Modules/SuperPower/SuperPower.hpp'
Assert-Contains $protocol 'FEEDBACK_ID = 0x051U' 'Feedback CAN ID must be 0x051.'
Assert-Contains $protocol 'COMMAND_ID = 0x061U' 'Command CAN ID must be 0x061.'
Assert-Contains $protocol 'float chassis_power' 'Feedback must carry float chassis power.'
Assert-Contains $protocol 'uint16_t referee_energy_buffer' 'Command must carry referee buffer.'
Assert-Contains $protocol 'sizeof(FeedbackData) == 8U' 'Feedback layout must be eight bytes.'
Assert-Contains $protocol 'sizeof(CommandData) == 8U' 'Command layout must be eight bytes.'
Assert-Contains $module 'COMMAND_PERIOD_MS = 5U' 'Command period must be 5 ms.'
Assert-Contains $module 'STATUS_RX_TIMEOUT_MS = 100U' 'Feedback timeout must be 100 ms.'
Assert-Contains $module 'Timer::CreateTask' 'CAN output must run from a timer.'
Assert-Contains $module 'referee_energy_buffer' 'Module must forward referee buffer.'
Assert-NotContains $module 'SAME_FRAME_OFFLINE_COUNT' 'Identical frames must remain online.'
Assert-NotContains $module 'POWER_ENCODE_OFFSET' 'RM2024 power is a float, not offset binary.'
Write-Output 'PASS: RM2024 SuperPower protocol static checks'
```

- [ ] **Step 2: Rewrite the README protocol section**

Document the exact layouts from Task 1, define `GetCapEnergy()` as stored energy normalized by 255, state that `0x061` derives both values from `chassis_ref`, and state that identical frames remain online while no feedback for 100 ms is offline.

- [ ] **Step 3: Run all checks and hardware acceptance**

Run the Task 1 executable, `pwsh -File tests/superpower_rm2024_protocol_static_regression.ps1`, `tools/format_code.sh --check`, and both Task 2 build commands.

On hardware, capture CAN and confirm a 5 ms `0x061` stream has byte 0 equal to `0x01`, little-endian referee power in bytes 1-2, and little-endian buffer energy in bytes 3-4. Confirm `0x051` reports float power and normalized energy, then verify state clears after 100 ms of disconnect and recovers after reconnect.

- [ ] **Step 4: Commit documentation and regression files**

Commit only `Modules/SuperPower/README.md` and `tests/superpower_rm2024_protocol_static_regression.ps1` with `git commit -m "docs: describe RM2024 superpower protocol"`.

## Plan Review

- Tasks 1 and 2 cover all reference-firmware bytes, send timing, data sources, and offline semantics.
- Existing PowerControl and chassis APIs remain compatible because capacitor energy stays normalized in `GetCapEnergy()`.
- Verification spans host byte vectors, static contract checks, embedded compilation, and a physical CAN trace.
