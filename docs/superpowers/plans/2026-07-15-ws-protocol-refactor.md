# WsProtocol Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the receive-only two-header WsProtocol implementation with a single-header, Referee-style bidirectional protocol module on the sentry gimbal board, including an independent 50 ms stale-command zero-output guard.

**Architecture:** `WsProtocol.hpp` owns all packed protocol types, the blocking UART receive loop, cached `Data`, Topic publication, and serialized transmit framing. A 1 ms LibXR global Timer task checks the 50 ms chassis-command deadline independently of UART blocking, while stale zero publications remain on a 50 ms cadence; one mutex orders recovered commands against stale zero publications. The gimbal configuration makes WsProtocol the only `chassis_data` producer, while the chassis board continues receiving the post-CMD command through DualBoard CAN.

**Tech Stack:** C++17, LibXR UART/Topic/Timer/Mutex/Thread, XRobot manifest and YAML generation, STM32F407, clang-format 21.1.8.

## Global Constraints

- `Modules/WsProtocol/WsProtocol.hpp` is the only production protocol header; delete `WsProtocolParser.hpp`.
- Preserve the existing `pldx_ws` v2 frame layout and do not modify `/home/sb/pldx_ws`.
- Keep PC-to-MCU and MCU-to-PC command IDs in separate `RxCommandID` and `TxCommandID` enums.
- Accept known RX payloads when `len >= sizeof(payload)` and ignore extra tail bytes.
- Do not add semantic range, `NaN`, or infinity validation.
- Use a fixed 261-byte maximum frame buffer; allocate only during initialization.
- Keep `Data` public, `data_` private, and do not add a getter.
- Do not register WsProtocol with `ApplicationManager`; keep `OnMonitor()` empty.
- Accept `thread_priority_uart` but create the RX thread at `MEDIUM` priority.
- Use a 50 ms UART read timeout and a 5000 ms UART write timeout.
- Use a 1 ms LibXR Timer check task and mutex so the 50 ms stale deadline is independent of blocking UART reads and 50 ms timer phase.
- Only a complete, CRC-valid `ROBOT_COMMAND` refreshes chassis freshness.
- From startup or the last valid command, detect expiry at 50 ms on the next 1 ms Timer tick and then publish zero `chassis_data` every 50 ms until recovery.
- Do not make WsProtocol aware of CMD mode and do not force chassis `RELAX`.
- Keep `chassis_data` non-retained.
- Do not add host protocol tests, PowerShell static tests, or layout `static_assert`s.
- Preserve user changes, especially the existing root `AGENTS.md` modification.
- Do not edit `User/xrobot_main.hpp`; it is ignored generated output.
- Do not modify `Drivers/`, `Middlewares/`, or `Core/` generated code.
- Increase USART6 TX backing storage from 512 to 528 bytes in both `User/libxr_config.yaml` and generated `User/app_main.cpp`, providing two 264-byte halves for a 261-byte maximum frame.
- Commit generated `User/app_main.cpp` separately from functional configuration, and commit module code separately because `Modules/*/` is ignored by the root repository.

---

## File Map

| Path | Repository | Action | Responsibility |
| --- | --- | --- | --- |
| `Modules/WsProtocol/WsProtocol.hpp` | WsProtocol module | Replace, then extend | Complete RX/TX protocol module |
| `Modules/WsProtocol/WsProtocolParser.hpp` | WsProtocol module | Delete | Remove obsolete byte-state parser |
| `Modules/WsProtocol/README.md` | WsProtocol module | Replace | Document public contract and double-board flow |
| `User/libxr_config.yaml` | root | Modify | Raise USART6 TX backing capacity to 528 bytes |
| `User/app_main.cpp` | root | Modify | Synchronize generated USART6 TX buffer storage |
| `User/RobotConfig/sentry_gimbal.yaml` | root | Modify | Instantiate WsProtocol on the gimbal board |
| `User/RobotConfig/sentry.yaml` | root | Modify | Restore legacy SharedTopic input after removing WsProtocol |
| `tests/ws_protocol_test.cpp` | root | Delete | Remove obsolete Parser host test |
| `tests/ws_protocol_static_regression.ps1` | root | Delete | Remove old text-bound architecture test |

### Task 1: Referee-Style Receive Path and Double-Board Integration

**Files:**
- Modify: `Modules/WsProtocol/WsProtocol.hpp`
- Delete: `Modules/WsProtocol/WsProtocolParser.hpp`
- Modify: `User/RobotConfig/sentry_gimbal.yaml:288`
- Modify: `User/RobotConfig/sentry.yaml:529`
- Delete: `tests/ws_protocol_test.cpp`
- Delete: `tests/ws_protocol_static_regression.ps1`

**Interfaces:**
- Consumes: `LibXR::UART`, `LibXR::CRC8`, `LibXR::CRC16`, `LibXR::Timer`, `LibXR::Topic`, `HostData::HostChassisTarget`.
- Produces: constructor `WsProtocol(hw, app, task_stack_depth_uart, uart, baudrate, chassis_topic_name, thread_priority_uart)` and a non-retained `chassis_data` publication.

- [ ] **Step 1: Attach the existing module directory to its authoritative Git history**

The local bytes currently match remote commit `a31927f`, but the directory has no `.git`. Initialize metadata without replacing source bytes:

```bash
git -C Modules/WsProtocol init -b main
git -C Modules/WsProtocol remote add origin https://github.com/GUET-PLDX/WsProtocol.git
git -C Modules/WsProtocol fetch origin main
git -C Modules/WsProtocol read-tree origin/main
git -C Modules/WsProtocol update-ref refs/heads/main refs/remotes/origin/main
git -C Modules/WsProtocol branch --set-upstream-to=origin/main main
git -C Modules/WsProtocol status --short --branch
git -C Modules/WsProtocol rev-parse --short HEAD
```

Expected: branch `main...origin/main`, no file changes, and HEAD `a31927f`. If status is not clean, stop and compare the local bytes with `origin/main` before editing.

- [ ] **Step 2: Replace `WsProtocol.hpp` with the complete receive-side implementation**

Use `apply_patch` to replace the file with this exact receive implementation. Task 3 adds transmit types and methods without changing this receive contract.

```cpp
#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: pldx_ws bidirectional UART protocol
constructor_args:
  - task_stack_depth_uart: 1024
  - uart: "uart_ext_controller"
  - baudrate: 115200
  - chassis_topic_name: "chassis_data"
  - thread_priority_uart: LibXR::Thread::Priority::LOW
template_args: []
required_hardware: uart
depends:
  - pldx/HostData
=== END MANIFEST === */
// clang-format on

#include <cstddef>
#include <cstdint>

#include "HostData.hpp"
#include "app_framework.hpp"
#include "crc.hpp"
#include "libxr_def.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "timer.hpp"
#include "uart.hpp"

class WsProtocol : public LibXR::Application {
 public:
  static constexpr uint8_t SOF = 0x5AU;
  static constexpr size_t MAX_PAYLOAD_SIZE = 255U;
  static constexpr size_t CRC16_SIZE = sizeof(uint16_t);
  static constexpr size_t MAX_FRAME_SIZE =
      sizeof(uint8_t) * 4U + MAX_PAYLOAD_SIZE + CRC16_SIZE;
  static constexpr uint32_t RX_TIMEOUT_MS = 50U;
  static constexpr uint32_t CHASSIS_COMMAND_TIMEOUT_MS = 50U;
  static constexpr uint32_t CHASSIS_WATCHDOG_CHECK_INTERVAL_MS = 1U;
  static constexpr uint32_t CHASSIS_ZERO_PUBLISH_INTERVAL_MS = 50U;

  enum class Status : uint8_t {
    OFFLINE = 0U,
    RUNNING,
  };

  enum class RxCommandID : uint8_t {
    ROBOT_COMMAND = 0x01U,
  };

  struct [[gnu::packed]] Header {
    uint8_t sof;
    uint8_t length;
    uint8_t id;
    uint8_t crc8;
  };

  struct [[gnu::packed]] SpeedVector {
    float vx;
    float vy;
    float wz;
  };

  struct [[gnu::packed]] ChassisCommand {
    float roll;
    float pitch;
    float yaw;
    float leg_length;
  };

  struct [[gnu::packed]] GimbalCommand {
    uint8_t control;
    float pitch;
    float yaw;
    float pitch_velocity;
    float yaw_velocity;
    float pitch_acceleration;
    float yaw_acceleration;
  };

  struct [[gnu::packed]] ShootCommand {
    uint8_t fire;
    uint8_t friction_wheels_on;
  };

  struct [[gnu::packed]] TrackingCommand {
    uint8_t tracking;
  };

  struct [[gnu::packed]] RobotCommandData {
    SpeedVector speed_vector;
    ChassisCommand chassis;
    GimbalCommand gimbal;
    ShootCommand shoot;
    TrackingCommand tracking;
  };

  struct [[gnu::packed]] RobotCommandPayload {
    uint32_t time_stamp;
    RobotCommandData data;
  };

  struct Data {
    Status status = Status::OFFLINE;
    RobotCommandPayload robot_command{};
  };

  WsProtocol(
      LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
      uint32_t task_stack_depth_uart, const char* uart, uint32_t baudrate,
      const char* chassis_topic_name,
      LibXR::Thread::Priority thread_priority_uart =
          LibXR::Thread::Priority::LOW)
      : uart_(hw.template FindOrExit<LibXR::UART>({uart})),
        read_sem_(0),
        read_op_(read_sem_, RX_TIMEOUT_MS),
        chassis_topic_(LibXR::Topic::CreateTopic<
                       HostData::HostChassisTarget>(chassis_topic_name)) {
    UNUSED(app);
    UNUSED(thread_priority_uart);

    uart_->SetConfig(
        {baudrate, LibXR::UART::Parity::NO_PARITY, 8U, 1U});

    startup_time_ms_ = NowMilliseconds();
    chassis_watchdog_ = LibXR::Timer::CreateTask(
        ChassisWatchdog, this, CHASSIS_WATCHDOG_CHECK_INTERVAL_MS);
    LibXR::Timer::Add(chassis_watchdog_);
    LibXR::Timer::Start(chassis_watchdog_);

    thread_.Create(this, ThreadFunc, "WsProtocol", task_stack_depth_uart,
                   LibXR::Thread::Priority::MEDIUM);
  }

  void OnMonitor() override {}

 private:
  static void ThreadFunc(WsProtocol* self) { self->Run(); }

  static void ChassisWatchdog(WsProtocol* self) {
    self->CheckChassisCommandFreshness();
  }

  static uint32_t NowMilliseconds() {
    return static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
  }

  void Run() {
    while (true) {
      FindHeader();
      last_parse_ = ParseData();
      if (last_parse_) {
        Publish();
      }
    }
  }

  void FindHeader() {
    while (true) {
      if (uart_->Read({&byte_, 1U}, read_op_) != LibXR::ErrorCode::OK) {
        data_.status = Status::OFFLINE;
        continue;
      }

      if (byte_ != SOF) {
        continue;
      }

      rx_frame_.header.sof = byte_;
      uart_->Read(
          {reinterpret_cast<uint8_t*>(&rx_frame_.header) + 1U,
           sizeof(Header) - 1U},
          read_op_);

      if (LibXR::CRC8::Verify(
              reinterpret_cast<uint8_t*>(&rx_frame_.header),
              sizeof(Header))) {
        data_.status = Status::RUNNING;
        return;
      }
    }
  }

  bool ParseData() {
    const size_t BYTES_AFTER_HEADER =
        static_cast<size_t>(rx_frame_.header.length) + CRC16_SIZE;
    if (BYTES_AFTER_HEADER > sizeof(rx_frame_.body)) {
      return false;
    }

    if (uart_->Read({rx_frame_.body, BYTES_AFTER_HEADER}, read_op_) !=
        LibXR::ErrorCode::OK) {
      return false;
    }

    if (!LibXR::CRC16::Verify(
            &rx_frame_, sizeof(Header) + BYTES_AFTER_HEADER)) {
      return false;
    }

    data_.status = Status::RUNNING;
    robot_command_parsed_ = false;
    switch (static_cast<RxCommandID>(rx_frame_.header.id)) {
      case RxCommandID::ROBOT_COMMAND: {
        if (rx_frame_.header.length < sizeof(RobotCommandPayload)) {
          return false;
        }

        LibXR::Memory::FastCopy(&data_.robot_command, rx_frame_.body,
                                sizeof(data_.robot_command));
        robot_command_parsed_ = true;
        return true;
      }

      default:
        return false;
    }
  }

  bool IsRobotCommandExpiredLocked(uint32_t now_ms) const {
    const uint32_t FRESHNESS_START_MS =
        robot_command_received_ ? last_robot_command_time_ms_
                                : startup_time_ms_;
    return now_ms - FRESHNESS_START_MS >= CHASSIS_COMMAND_TIMEOUT_MS;
  }

  void Publish() {
    LibXR::Mutex::LockGuard lock(chassis_mutex_);
    const uint32_t NOW_MS = NowMilliseconds();
    if (robot_command_parsed_) {
      last_robot_command_time_ms_ = NOW_MS;
      robot_command_received_ = true;
      stale_zero_published_ = false;
      robot_command_parsed_ = false;
    }

    PublishChassisTargetLocked(NOW_MS);
  }

  void PublishChassisTargetLocked(uint32_t now_ms) {
    const bool ROBOT_COMMAND_EXPIRED =
        IsRobotCommandExpiredLocked(now_ms);
    HostData::HostChassisTarget chassis{};
    if (robot_command_received_ && !ROBOT_COMMAND_EXPIRED) {
      chassis.vx = data_.robot_command.data.speed_vector.vx;
      chassis.vy = data_.robot_command.data.speed_vector.vy;
      chassis.w = data_.robot_command.data.speed_vector.wz;
      chassis_topic_.Publish(chassis);
      return;
    }

    chassis_topic_.Publish(chassis);
    if (ROBOT_COMMAND_EXPIRED) {
      last_zero_publish_time_ms_ = now_ms;
      stale_zero_published_ = true;
    }
  }

  void CheckChassisCommandFreshness() {
    LibXR::Mutex::LockGuard lock(chassis_mutex_);
    const uint32_t NOW_MS = NowMilliseconds();
    if (!IsRobotCommandExpiredLocked(NOW_MS)) {
      return;
    }

    if (stale_zero_published_ &&
        NOW_MS - last_zero_publish_time_ms_ <
            CHASSIS_ZERO_PUBLISH_INTERVAL_MS) {
      return;
    }

    HostData::HostChassisTarget chassis{};
    chassis_topic_.Publish(chassis);
    last_zero_publish_time_ms_ = NOW_MS;
    stale_zero_published_ = true;
  }

  struct [[gnu::packed]] ReceiveFrame {
    Header header;
    uint8_t body[MAX_PAYLOAD_SIZE + CRC16_SIZE];
  };

  LibXR::UART* uart_;
  LibXR::Semaphore read_sem_;
  LibXR::ReadOperation read_op_;
  LibXR::Mutex chassis_mutex_;
  LibXR::Thread thread_;
  LibXR::Timer::TimerHandle chassis_watchdog_ = nullptr;
  LibXR::Topic chassis_topic_;

  ReceiveFrame rx_frame_{};
  Data data_{};
  uint8_t byte_ = 0U;
  uint32_t startup_time_ms_ = 0U;
  uint32_t last_robot_command_time_ms_ = 0U;
  uint32_t last_zero_publish_time_ms_ = 0U;
  bool robot_command_received_ = false;
  bool robot_command_parsed_ = false;
  bool stale_zero_published_ = false;
  bool last_parse_ = false;
};
```

- [ ] **Step 3: Delete the obsolete parser header**

Use `apply_patch`:

```diff
*** Begin Patch
*** Delete File: Modules/WsProtocol/WsProtocolParser.hpp
*** End Patch
```

Run:

```bash
! rg -n "WsProtocolParser|parser_\.Push" Modules/WsProtocol
```

Expected: exit zero after negation and no matches.

- [ ] **Step 4: Move the production instance to the gimbal configuration**

Use `apply_patch` with the current YAML context. This moves the UART owner to
the production gimbal configuration, removes the competing local SharedTopic
producer there, and restores the unused single-board configuration's original
SharedTopic input:

```diff
*** Begin Patch
*** Update File: User/RobotConfig/sentry_gimbal.yaml
@@
 - id: hostdata
   name: HostData
   constructor_args:
     cmd: '@cmd'
     host_euler_topic_name: target_euler
     host_chassis_data_topic_name: chassis_data
     host_fire_topic_name: fire_notify
+- id: ws_protocol
+  name: WsProtocol
+  constructor_args:
+    task_stack_depth_uart: 1024
+    uart: uart_ext_controller
+    baudrate: 115200
+    chassis_topic_name: chassis_data
+    thread_priority_uart: LibXR::Thread::Priority::MEDIUM
 - id: sharetopic
   name: SharedTopic
   constructor_args:
     uart_name: usb_otg_hs_cdc
     buffer_size: 256
     topic_configs:
-    - chassis_data
     - sentry_state
     - target_euler
     - fire_notify
*** Update File: User/RobotConfig/sentry.yaml
@@
 - id: hostdata
   name: HostData
   constructor_args:
     cmd: '@cmd'
     host_euler_topic_name: target_euler
     host_chassis_data_topic_name: chassis_data
     host_fire_topic_name: fire_notify
-- id: ws_protocol
-  name: WsProtocol
-  constructor_args:
-    uart_name: uart_ext_controller
-    chassis_topic_name: chassis_data
-    task_stack_depth: 1024
-    thread_priority: LibXR::Thread::Priority::MEDIUM
 - id: sharetopic
   name: SharedTopic
   constructor_args:
     uart_name: usb_otg_hs_cdc
     buffer_size: 256
     topic_configs:
+    - chassis_data
     - sentry_state
     - target_euler
     - fire_notify
*** End Patch
```

Expected checks:

```bash
rg -n "id: ws_protocol|uart: uart_ext_controller|chassis_data" User/RobotConfig/sentry_gimbal.yaml
rg -n "id: ws_protocol|chassis_data" User/RobotConfig/sentry.yaml
python3 - <<'PY'
from pathlib import Path

import yaml


def load_modules(path):
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return document["modules"]


gimbal_modules = load_modules("User/RobotConfig/sentry_gimbal.yaml")
sentry_modules = load_modules("User/RobotConfig/sentry.yaml")

assert sum(module["id"] == "ws_protocol" for module in gimbal_modules) == 1
assert sum(module["id"] == "ws_protocol" for module in sentry_modules) == 0

gimbal_by_id = {module["id"]: module for module in gimbal_modules}
sentry_by_id = {module["id"]: module for module in sentry_modules}
assert gimbal_by_id["ws_protocol"]["constructor_args"] == {
    "task_stack_depth_uart": 1024,
    "uart": "uart_ext_controller",
    "baudrate": 115200,
    "chassis_topic_name": "chassis_data",
    "thread_priority_uart": "LibXR::Thread::Priority::MEDIUM",
}
assert "chassis_data" not in (
    gimbal_by_id["sharetopic"]["constructor_args"]["topic_configs"]
)
assert "chassis_data" in (
    sentry_by_id["sharetopic"]["constructor_args"]["topic_configs"]
)
PY
```

Expected: both searches show the intended human-readable context, and the
Python YAML assertions exit zero. WsProtocol is the gimbal ingress publisher;
the single-board configuration instead receives `chassis_data` through
SharedTopic.

- [ ] **Step 5: Delete tests tied to the removed parser architecture**

Use `apply_patch` to delete both files:

```diff
*** Begin Patch
*** Delete File: tests/ws_protocol_test.cpp
*** Delete File: tests/ws_protocol_static_regression.ps1
*** End Patch
```

Run:

```bash
! rg --files tests | rg 'ws_protocol'
```

Expected: exit zero after negation and no output.

- [ ] **Step 6: Format only the changed module header**

Run:

```bash
clang-format --version
clang-format -i Modules/WsProtocol/WsProtocol.hpp
clang-format --dry-run --Werror Modules/WsProtocol/WsProtocol.hpp
```

Expected: clang-format 21.1.8 exits zero. Do not run an apply-format command over unrelated modules.

- [ ] **Step 7: Build the actual sentry gimbal target**

Run:

```bash
tools/build.sh --skip-format \
  -c User/RobotConfig/sentry_gimbal.yaml \
  -b build/ws_protocol_sentry_gimbal
```

Expected: XRobot generation accepts the new constructor order and the firmware
links successfully with `-Werror`. `User/xrobot_main.hpp` may be regenerated,
but it is ignored and must not be staged.

- [ ] **Step 8: Commit the receive module in its own repository**

Run:

```bash
git -C Modules/WsProtocol diff --check
git -C Modules/WsProtocol status --short
git -C Modules/WsProtocol add WsProtocol.hpp WsProtocolParser.hpp
git -C Modules/WsProtocol diff --cached --name-only
git -C Modules/WsProtocol commit -m "feat: refactor ws protocol receive path"
```

Expected staged names before commit: only `WsProtocol.hpp` and the deleted
`WsProtocolParser.hpp`.

- [ ] **Step 9: Commit only root configuration and obsolete-test cleanup**

Run:

```bash
git add User/RobotConfig/sentry_gimbal.yaml User/RobotConfig/sentry.yaml \
  tests/ws_protocol_test.cpp tests/ws_protocol_static_regression.ps1
git diff --cached --check
git diff --cached --name-only
git commit -m "config: route sentry navigation through ws protocol"
```

Expected: the staged list contains exactly the four root paths above. Do not
stage `AGENTS.md`, generated files, build output, or anything under
`Modules/WsProtocol` from the root repository.

### Task 2: Provision USART6 for Maximum Protocol Frames

**Files:**
- Modify: `User/libxr_config.yaml:36`
- Modify: `User/app_main.cpp:58`

**Interfaces:**
- Consumes: the protocol-wide 261-byte maximum frame size.
- Produces: a 528-byte USART6 TX backing store split by LibXR into two 264-byte halves, so one raw `SendFrame()` write can contain the full frame.

- [ ] **Step 1: Increase the USART6 TX capacity in its source configuration**

Use `apply_patch`:

```diff
*** Begin Patch
*** Update File: User/libxr_config.yaml
@@
   usart6:
-    tx_buffer_size: 512
+    tx_buffer_size: 528
     rx_buffer_size: 512
     tx_queue_size: 15
*** End Patch
```

The RX size stays 512. A 528-byte TX backing is the smallest value that both
provides at least 261 bytes per half and satisfies LibXR `DoubleBuffer`'s
`2 * alignof(size_t)` size multiple on Cortex-M4.

- [ ] **Step 2: Synchronize the generated hardware map**

Use `apply_patch` for the single generated value:

```diff
*** Begin Patch
*** Update File: User/app_main.cpp
@@
-static uint8_t usart6_tx_buf[512];
+static uint8_t usart6_tx_buf[528];
 static uint8_t usart6_rx_buf[512];
*** End Patch
```

Do not change any other generated declaration or code in `User/app_main.cpp`.

- [ ] **Step 3: Verify the configured and generated capacities agree**

Run:

```bash
rg -n -A3 "^  usart6:" User/libxr_config.yaml
rg -n "usart6_tx_buf\[528\]|usart6_rx_buf\[512\]" User/app_main.cpp
```

Expected: source config reports TX 528/RX 512, and the generated map reports
the same two sizes. Since `STM32UART` divides TX storage by two, the resulting
264-byte write capacity exceeds `MAX_FRAME_SIZE == 261`.

- [ ] **Step 4: Commit only the functional hardware configuration**

Run:

```bash
git add User/libxr_config.yaml
git diff --cached --check
git diff --cached --name-only
git commit -m "config: enlarge usart6 transmit buffer"
```

Expected staged name before commit: only `User/libxr_config.yaml`. Keep the
generated hardware map out of this functional configuration commit.

- [ ] **Step 5: Commit only the synchronized generated hardware map**

Run:

```bash
git add User/app_main.cpp
git diff --cached --check
git diff --cached --name-only
git commit -m "chore: sync usart6 generated buffer"
```

Expected staged name before commit: only `User/app_main.cpp`.

- [ ] **Step 6: Rebuild with the enlarged hardware buffer**

Run:

```bash
tools/build.sh --skip-format \
  -c User/RobotConfig/sentry_gimbal.yaml \
  -b build/ws_protocol_sentry_gimbal
```

Expected: the sentry gimbal firmware still links with `-Werror`, and the map
uses the 528-byte `usart6_tx_buf` declaration.

### Task 3: Complete MCU-to-PC Transmit Protocol

**Files:**
- Modify: `Modules/WsProtocol/WsProtocol.hpp`

**Interfaces:**
- Consumes: Task 1 `Header`, `SpeedVector`, UART, `MAX_PAYLOAD_SIZE`, and `MAX_FRAME_SIZE`, plus Task 2's 264-byte per-write USART6 capacity.
- Produces:
  - `SendFrame(TxCommandID, const void*, uint16_t) -> LibXR::ErrorCode`;
  - `SendFrame<T>(TxCommandID, const T&) -> LibXR::ErrorCode`;
  - one `Send*()` wrapper for every `TxCommandID` from `0x01` through `0x0E`.

- [ ] **Step 1: Add all transmit IDs and exact business/payload types**

Insert only `TxCommandID` immediately after `RxCommandID`:

```cpp
  enum class TxCommandID : uint8_t {
    DEBUG = 0x01U,
    IMU = 0x02U,
    ROBOT_STATE_INFO = 0x03U,
    EVENT_DATA = 0x04U,
    PID_DEBUG = 0x05U,
    ALL_ROBOT_HP = 0x06U,
    GAME_STATUS = 0x07U,
    ROBOT_MOTION = 0x08U,
    GROUND_ROBOT_POSITION = 0x09U,
    RFID_STATUS = 0x0AU,
    ROBOT_STATUS = 0x0BU,
    JOINT_STATE = 0x0CU,
    BUFF = 0x0DU,
    GIMBAL_STATE = 0x0EU,
  };
```

Then insert the remaining public constants and wire types immediately after
`RobotCommandPayload` and before `Data`. This placement keeps the shared
`SpeedVector` declaration ahead of `RobotMotionData`. Keep the order shown so
the wire models can be audited against `pldx_ws/packet_typedef.hpp` linearly:

```cpp

  static constexpr size_t DEBUG_PACKAGE_COUNT = 10U;
  static constexpr size_t DEBUG_PACKAGE_NAME_LENGTH = 10U;

  struct [[gnu::packed]] DebugPackage {
    uint8_t name[DEBUG_PACKAGE_NAME_LENGTH];
    uint8_t type;
    float data;
  };

  struct [[gnu::packed]] DebugData {
    DebugPackage packages[DEBUG_PACKAGE_COUNT];
  };

  struct [[gnu::packed]] DebugPayload {
    uint32_t time_stamp;
    DebugPackage packages[DEBUG_PACKAGE_COUNT];
  };

  struct [[gnu::packed]] ImuData {
    float yaw;
    float pitch;
    float roll;
    float yaw_velocity;
    float pitch_velocity;
    float roll_velocity;
  };

  struct [[gnu::packed]] ImuPayload {
    uint32_t time_stamp;
    ImuData data;
  };

  struct [[gnu::packed]] RobotPartType {
    uint16_t chassis : 3;
    uint16_t gimbal : 3;
    uint16_t shoot : 3;
    uint16_t arm : 3;
    uint16_t custom_controller : 3;
    uint16_t reserved : 1;
  };

  struct [[gnu::packed]] RobotPartState {
    uint8_t chassis : 1;
    uint8_t gimbal : 1;
    uint8_t shoot : 1;
    uint8_t arm : 1;
    uint8_t custom_controller : 1;
    uint8_t reserved : 3;
  };

  struct [[gnu::packed]] RobotStateInfoData {
    RobotPartType type;
    RobotPartState state;
  };

  struct [[gnu::packed]] RobotStateInfoPayload {
    uint32_t time_stamp;
    RobotStateInfoData data;
  };

  struct [[gnu::packed]] EventData {
    uint8_t non_overlapping_supply_zone : 1;
    uint8_t overlapping_supply_zone : 1;
    uint8_t supply_zone : 1;
    uint8_t small_energy : 1;
    uint8_t big_energy : 1;
    uint8_t central_highland : 2;
    uint8_t reserved_1 : 1;
    uint8_t trapezoidal_highland : 2;
    uint8_t center_gain_zone : 2;
    uint8_t reserved_2 : 4;
  };

  struct [[gnu::packed]] EventPayload {
    uint32_t time_stamp;
    EventData data;
  };

  struct [[gnu::packed]] PidDebugData {
    float feedback;
    float reference;
    float output;
  };

  struct [[gnu::packed]] PidDebugPayload {
    uint32_t time_stamp;
    PidDebugData data;
  };

  struct [[gnu::packed]] AllRobotHpData {
    uint16_t red_1_robot_hp;
    uint16_t red_2_robot_hp;
    uint16_t red_3_robot_hp;
    uint16_t red_4_robot_hp;
    uint16_t red_7_robot_hp;
    uint16_t red_outpost_hp;
    uint16_t red_base_hp;
    uint16_t blue_1_robot_hp;
    uint16_t blue_2_robot_hp;
    uint16_t blue_3_robot_hp;
    uint16_t blue_4_robot_hp;
    uint16_t blue_7_robot_hp;
    uint16_t blue_outpost_hp;
    uint16_t blue_base_hp;
  };

  struct [[gnu::packed]] AllRobotHpPayload {
    uint32_t time_stamp;
    AllRobotHpData data;
  };

  struct [[gnu::packed]] GameStatusData {
    uint8_t game_progress;
    uint16_t stage_remaining_time;
  };

  struct [[gnu::packed]] GameStatusPayload {
    uint32_t time_stamp;
    GameStatusData data;
  };

  struct [[gnu::packed]] RobotMotionData {
    SpeedVector speed_vector;
  };

  struct [[gnu::packed]] RobotMotionPayload {
    uint32_t time_stamp;
    RobotMotionData data;
  };

  struct [[gnu::packed]] GroundRobotPositionData {
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved_1;
    float reserved_2;
  };

  struct [[gnu::packed]] GroundRobotPositionPayload {
    uint32_t time_stamp;
    GroundRobotPositionData data;
  };

  struct [[gnu::packed]] RfidStatusData {
    uint32_t base_gain_point : 1;
    uint32_t central_highland_gain_point : 1;
    uint32_t enemy_central_highland_gain_point : 1;
    uint32_t friendly_trapezoidal_highland_gain_point : 1;
    uint32_t enemy_trapezoidal_highland_gain_point : 1;
    uint32_t friendly_fly_ramp_front_gain_point : 1;
    uint32_t friendly_fly_ramp_back_gain_point : 1;
    uint32_t enemy_fly_ramp_front_gain_point : 1;
    uint32_t enemy_fly_ramp_back_gain_point : 1;
    uint32_t friendly_central_highland_lower_gain_point : 1;
    uint32_t friendly_central_highland_upper_gain_point : 1;
    uint32_t enemy_central_highland_lower_gain_point : 1;
    uint32_t enemy_central_highland_upper_gain_point : 1;
    uint32_t friendly_highway_lower_gain_point : 1;
    uint32_t friendly_highway_upper_gain_point : 1;
    uint32_t enemy_highway_lower_gain_point : 1;
    uint32_t enemy_highway_upper_gain_point : 1;
    uint32_t friendly_fortress_gain_point : 1;
    uint32_t friendly_outpost_gain_point : 1;
    uint32_t friendly_supply_zone_non_exchange : 1;
    uint32_t friendly_supply_zone_exchange : 1;
    uint32_t friendly_big_resource_island : 1;
    uint32_t enemy_big_resource_island : 1;
    uint32_t center_gain_point : 1;
    uint32_t reserved : 8;
  };

  struct [[gnu::packed]] RfidStatusPayload {
    uint32_t time_stamp;
    RfidStatusData data;
  };

  struct [[gnu::packed]] RobotStatusData {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t shooter_17mm_1_barrel_heat;
    float robot_position_x;
    float robot_position_y;
    float robot_position_angle;
    uint8_t armor_id : 4;
    uint8_t hp_deduction_reason : 4;
    uint16_t projectile_allowance_17mm;
    uint16_t remaining_gold_coin;
  };

  struct [[gnu::packed]] RobotStatusPayload {
    uint32_t time_stamp;
    RobotStatusData data;
  };

  struct [[gnu::packed]] JointStateData {
    float pitch;
    float yaw;
  };

  struct [[gnu::packed]] JointStatePayload {
    uint32_t time_stamp;
    JointStateData data;
  };

  struct [[gnu::packed]] BuffData {
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
  };

  struct [[gnu::packed]] BuffPayload {
    uint32_t time_stamp;
    BuffData data;
  };

  struct [[gnu::packed]] GimbalStateData {
    uint8_t mode;
    float pitch;
    float yaw;
    float pitch_velocity;
    float yaw_velocity;
    float bullet_speed;
    uint16_t bullet_count;
  };

  struct [[gnu::packed]] GimbalStatePayload {
    uint32_t time_stamp;
    GimbalStateData data;
  };
```

- [ ] **Step 2: Add generic framing and every named send wrapper**

Add `TX_TIMEOUT_MS` beside the existing public constants:

```cpp
  static constexpr uint32_t TX_TIMEOUT_MS = 5000U;
```

Add these public methods before `OnMonitor()`:

```cpp
  LibXR::ErrorCode SendFrame(TxCommandID command_id, const void* payload,
                             uint16_t payload_len) {
    LibXR::Mutex::LockGuard lock(tx_mutex_);
    if (payload_len > MAX_PAYLOAD_SIZE ||
        (payload_len > 0U && payload == nullptr)) {
      return LibXR::ErrorCode::ARG_ERR;
    }

    Header header{};
    header.sof = SOF;
    header.length = static_cast<uint8_t>(payload_len);
    header.id = static_cast<uint8_t>(command_id);
    header.crc8 = LibXR::CRC8::Calculate(&header, sizeof(Header) - 1U);

    size_t offset = 0U;
    LibXR::Memory::FastCopy(&tx_buffer_[offset], &header, sizeof(header));
    offset += sizeof(header);

    if (payload_len > 0U) {
      LibXR::Memory::FastCopy(&tx_buffer_[offset], payload, payload_len);
      offset += payload_len;
    }

    const uint16_t CRC = LibXR::CRC16::Calculate(tx_buffer_, offset);
    tx_buffer_[offset++] = static_cast<uint8_t>(CRC & 0x00FFU);
    tx_buffer_[offset++] = static_cast<uint8_t>((CRC >> 8U) & 0x00FFU);
    return uart_->Write({tx_buffer_, offset}, write_op_);
  }

  template <typename PayloadType>
  LibXR::ErrorCode SendFrame(TxCommandID command_id,
                             const PayloadType& payload) {
    if (sizeof(PayloadType) > MAX_PAYLOAD_SIZE) {
      return LibXR::ErrorCode::ARG_ERR;
    }

    return SendFrame(command_id, &payload,
                     static_cast<uint16_t>(sizeof(PayloadType)));
  }

  LibXR::ErrorCode SendDebugData(const DebugData& data) {
    DebugPayload payload{};
    payload.time_stamp = NowMilliseconds();
    LibXR::Memory::FastCopy(payload.packages, data.packages,
                            sizeof(payload.packages));
    return SendFrame(TxCommandID::DEBUG, payload);
  }

  LibXR::ErrorCode SendImuData(const ImuData& data) {
    ImuPayload payload{};
    payload.time_stamp = NowMilliseconds();
    payload.data = data;
    return SendFrame(TxCommandID::IMU, payload);
  }

  LibXR::ErrorCode SendRobotStateInfo(const RobotStateInfoData& data) {
    RobotStateInfoPayload payload{};
    payload.time_stamp = NowMilliseconds();
    payload.data = data;
    return SendFrame(TxCommandID::ROBOT_STATE_INFO, payload);
  }

  LibXR::ErrorCode SendEventData(const EventData& data) {
    EventPayload payload{};
    payload.time_stamp = NowMilliseconds();
    payload.data = data;
    return SendFrame(TxCommandID::EVENT_DATA, payload);
  }

  LibXR::ErrorCode SendPidDebugData(const PidDebugData& data) {
    PidDebugPayload payload{};
    payload.time_stamp = NowMilliseconds();
    payload.data = data;
    return SendFrame(TxCommandID::PID_DEBUG, payload);
  }

  LibXR::ErrorCode SendAllRobotHp(const AllRobotHpData& data) {
    AllRobotHpPayload payload{};
    payload.time_stamp = NowMilliseconds();
    payload.data = data;
    return SendFrame(TxCommandID::ALL_ROBOT_HP, payload);
  }

  LibXR::ErrorCode SendGameStatus(const GameStatusData& data) {
    GameStatusPayload payload{};
    payload.time_stamp = NowMilliseconds();
    payload.data = data;
    return SendFrame(TxCommandID::GAME_STATUS, payload);
  }

  LibXR::ErrorCode SendRobotMotion(const RobotMotionData& data) {
    RobotMotionPayload payload{};
    payload.time_stamp = NowMilliseconds();
    payload.data = data;
    return SendFrame(TxCommandID::ROBOT_MOTION, payload);
  }

  LibXR::ErrorCode SendGroundRobotPosition(
      const GroundRobotPositionData& data) {
    GroundRobotPositionPayload payload{};
    payload.time_stamp = NowMilliseconds();
    payload.data = data;
    return SendFrame(TxCommandID::GROUND_ROBOT_POSITION, payload);
  }

  LibXR::ErrorCode SendRfidStatus(const RfidStatusData& data) {
    RfidStatusPayload payload{};
    payload.time_stamp = NowMilliseconds();
    payload.data = data;
    return SendFrame(TxCommandID::RFID_STATUS, payload);
  }

  LibXR::ErrorCode SendRobotStatus(const RobotStatusData& data) {
    RobotStatusPayload payload{};
    payload.time_stamp = NowMilliseconds();
    payload.data = data;
    return SendFrame(TxCommandID::ROBOT_STATUS, payload);
  }

  LibXR::ErrorCode SendJointState(const JointStateData& data) {
    JointStatePayload payload{};
    payload.time_stamp = NowMilliseconds();
    payload.data = data;
    return SendFrame(TxCommandID::JOINT_STATE, payload);
  }

  LibXR::ErrorCode SendBuff(const BuffData& data) {
    BuffPayload payload{};
    payload.time_stamp = NowMilliseconds();
    payload.data = data;
    return SendFrame(TxCommandID::BUFF, payload);
  }

  LibXR::ErrorCode SendGimbalState(const GimbalStateData& data) {
    GimbalStatePayload payload{};
    payload.time_stamp = NowMilliseconds();
    payload.data = data;
    return SendFrame(TxCommandID::GIMBAL_STATE, payload);
  }
```

- [ ] **Step 3: Add transmit synchronization and buffer state**

Extend the constructor initializer list immediately after `read_op_`:

```cpp
        write_sem_(0),
        write_op_(write_sem_, TX_TIMEOUT_MS),
```

Add these members immediately after the receive operation members:

```cpp
  LibXR::Semaphore write_sem_;
  LibXR::WriteOperation write_op_;
  LibXR::Mutex tx_mutex_;
  uint8_t tx_buffer_[MAX_FRAME_SIZE]{};
```

Do not add retries, a TX queue, sequence fields, or internal rate scheduling.

- [ ] **Step 4: Format and build the complete bidirectional module**

Run:

```bash
clang-format --version
clang-format -i Modules/WsProtocol/WsProtocol.hpp
clang-format --dry-run --Werror Modules/WsProtocol/WsProtocol.hpp
tools/build.sh --skip-format \
  -c User/RobotConfig/sentry_gimbal.yaml \
  -b build/ws_protocol_sentry_gimbal
```

Expected: all commands compile with C++17 and `-Werror`; firmware links.

- [ ] **Step 5: Audit every transmit ID and wrapper before committing**

Run:

```bash
rg -n "DEBUG =|IMU =|ROBOT_STATE_INFO =|EVENT_DATA =|PID_DEBUG =|ALL_ROBOT_HP =|GAME_STATUS =|ROBOT_MOTION =|GROUND_ROBOT_POSITION =|RFID_STATUS =|ROBOT_STATUS =|JOINT_STATE =|BUFF =|GIMBAL_STATE =" Modules/WsProtocol/WsProtocol.hpp
rg -n "SendDebugData|SendImuData|SendRobotStateInfo|SendEventData|SendPidDebugData|SendAllRobotHp|SendGameStatus|SendRobotMotion|SendGroundRobotPosition|SendRfidStatus|SendRobotStatus|SendJointState|SendBuff|SendGimbalState" Modules/WsProtocol/WsProtocol.hpp
test "$(rg -c '^    (DEBUG|IMU|ROBOT_STATE_INFO|EVENT_DATA|PID_DEBUG|ALL_ROBOT_HP|GAME_STATUS|ROBOT_MOTION|GROUND_ROBOT_POSITION|RFID_STATUS|ROBOT_STATUS|JOINT_STATE|BUFF|GIMBAL_STATE) = 0x' Modules/WsProtocol/WsProtocol.hpp)" -eq 14
test "$(rg -c '^  LibXR::ErrorCode Send(DebugData|ImuData|RobotStateInfo|EventData|PidDebugData|AllRobotHp|GameStatus|RobotMotion|GroundRobotPosition|RfidStatus|RobotStatus|JointState|Buff|GimbalState)\(' Modules/WsProtocol/WsProtocol.hpp)" -eq 14
git -C Modules/WsProtocol diff --check
```

Expected: exactly 14 enum entries and 14 named wrappers, with IDs `0x01` to
`0x0E` in order.

- [ ] **Step 6: Commit the transmit API in the module repository**

Run:

```bash
git -C Modules/WsProtocol add WsProtocol.hpp
git -C Modules/WsProtocol diff --cached --name-only
git -C Modules/WsProtocol commit -m "feat: add ws protocol transmit frames"
```

Expected staged name: only `WsProtocol.hpp`.

### Task 4: Module Documentation and Completion Audit

**Files:**
- Modify: `Modules/WsProtocol/README.md`

**Interfaces:**
- Consumes: final APIs and behavior from Tasks 1 through 3.
- Produces: user-facing module contract and final verification evidence.

- [ ] **Step 1: Replace the module README with the final operational contract**

Use `apply_patch` to replace `Modules/WsProtocol/README.md` with:

````markdown
# WsProtocol

`WsProtocol` implements the existing `pldx_ws` version 2 UART protocol on the
sentry gimbal board. It receives complete robot commands, publishes chassis
velocity to `HostData`, and provides MCU-to-PC send methods for every command
currently defined by `pldx_ws`.

## Double-Board Data Path

```text
pldx_ws
  -> uart_ext_controller
  -> gimbal WsProtocol
  -> chassis_data
  -> HostData / CMD / chassis_cmd
  -> DualBoard CAN
  -> chassis Chassis
```

`chassis_data` is local to the gimbal board. The gimbal-to-chassis wire message
is the DualBoard CAN control frame produced after CMD arbitration.

## Receive Behavior

- UART frame: `0x5A | len | id | CRC8 | payload | CRC16`.
- RX and TX support the protocol maximum payload length of 255 bytes.
- `ROBOT_COMMAND (0x01)` caches timestamp, speed, chassis, gimbal, shoot, and
  tracking fields.
- Only `speed_vector.vx`, `vy`, and `wz` are currently published to the
  non-retained `chassis_data` Topic.
- Known payloads accept `len >= sizeof(payload)` and ignore extra tail bytes.
- Unknown IDs and failed reads or CRC checks do not update command data.

## Stale Command Behavior

Only a complete, CRC-valid `ROBOT_COMMAND` refreshes chassis freshness. A
1 ms LibXR Timer task detects the 50 ms deadline on the next scheduler tick.
It publishes zero `chassis_data` on first expiry and every 50 ms thereafter
until a new valid command arrives. This guard is independent of UART reads and
does not change CMD mode or chassis mode.

## Send Behavior

The module exposes raw and typed `SendFrame()` overloads plus named methods for
DEBUG, IMU, ROBOT_STATE_INFO, EVENT_DATA, PID_DEBUG, ALL_ROBOT_HP, GAME_STATUS,
ROBOT_MOTION, GROUND_ROBOT_POSITION, RFID_STATUS, ROBOT_STATUS, JOINT_STATE,
BUFF, and GIMBAL_STATE.

Named methods add the current LibXR millisecond timestamp. Calls write
immediately, return `LibXR::ErrorCode`, and are serialized by a TX mutex. The
caller owns send frequency; the module does not queue, retry, or rate-limit.
The sentry USART6 hardware map provides 264 bytes per write, covering the
maximum 261-byte frame.

## Constructor Arguments

- `task_stack_depth_uart`: receive thread stack depth.
- `uart`: hardware alias; the sentry gimbal uses `uart_ext_controller`.
- `baudrate`: UART baud rate; `pldx_ws` uses 115200.
- `chassis_topic_name`: `HostData::HostChassisTarget` Topic name.
- `thread_priority_uart`: retained for Referee-compatible configuration; the
  implementation creates the receive thread at `MEDIUM` priority.

## Dependencies

- `pldx/HostData`
````

- [ ] **Step 2: Commit the README in the module repository**

Run:

```bash
git -C Modules/WsProtocol add README.md
git -C Modules/WsProtocol diff --cached --check
git -C Modules/WsProtocol diff --cached --name-only
git -C Modules/WsProtocol commit -m "docs: document ws protocol data flow"
```

Expected staged name: only `README.md`.

- [ ] **Step 3: Run the approved automated verification**

Run:

```bash
tools/format_code.sh --check
tools/build.sh --skip-format \
  -c User/RobotConfig/sentry_gimbal.yaml \
  -b build/ws_protocol_sentry_gimbal
git diff --check
git -C Modules/WsProtocol diff --check
```

Expected: the sentry gimbal firmware builds and both diff checks report no
whitespace errors. The repository-wide format check either passes, or reports
only an explicitly recorded pre-existing failure outside the changed module;
the changed header's Task 3 per-file check must pass with clang-format 21.1.8.

If the repository-wide format check reports a pre-existing failure in an
unrelated module, record the exact path and diagnostic in the execution report.
Do not run repository-wide apply-format or modify that unrelated module; the
per-file `clang-format --dry-run --Werror Modules/WsProtocol/WsProtocol.hpp`
check from Task 3 must still pass.

- [ ] **Step 4: Prove configuration ownership and removed architecture**

Run:

```bash
rg -n "id: ws_protocol" User/RobotConfig
rg -n -A12 "id: sharetopic" User/RobotConfig/sentry_gimbal.yaml User/RobotConfig/sentry.yaml
rg -n -A3 "^  usart6:" User/libxr_config.yaml
rg -n "usart6_tx_buf\[528\]" User/app_main.cpp
test ! -e Modules/WsProtocol/WsProtocolParser.hpp
test ! -e tests/ws_protocol_test.cpp
test ! -e tests/ws_protocol_static_regression.ps1
git check-ignore User/xrobot_main.hpp build/ws_protocol_sentry_gimbal
```

Expected:

- only `sentry_gimbal.yaml` instantiates WsProtocol;
- gimbal SharedTopic does not list `chassis_data`;
- single-board sentry SharedTopic does list `chassis_data`;
- USART6 source and generated TX capacities are both 528 bytes;
- all three obsolete files are absent;
- generated header and build directory are ignored.

- [ ] **Step 5: Complete the manual hardware integration gate**

Use the unchanged `pldx_ws`, the sentry gimbal board, and debugger or serial
capture to record each result:

1. Send a complete 60-byte `ROBOT_COMMAND` whose timestamp, speed, chassis,
   gimbal, shoot, and tracking fields each contain distinct known values.
   Inspect `data_.robot_command` in the debugger to confirm every field, order,
   and width matches `pldx_ws`, then confirm `vx`, `vy`, and `wz` reach the
   chassis over the gimbal-to-chassis CAN path.
2. Stop valid robot commands at a recorded timestamp. Confirm the first zero
   output occurs on the first 1 ms watchdog tick at or after 50 ms, then
   continues at 50 ms intervals without changing CMD or chassis mode.
3. While stale, send partial frames, bad CRC8/CRC16 frames, and valid traffic
   with unrelated IDs. Confirm none restores the old chassis velocity.
4. Resume a valid `ROBOT_COMMAND` and confirm its velocity is the next
   published chassis command, with no later stale-zero overwrite.
5. Invoke all 14 named MCU-to-PC methods through the debugger or a temporary
   uncommitted hardware-debug call site. Confirm all frames have the defined
   ID, length, CRC, timestamp, and payload bytes. Use the unchanged `pldx_ws`
   handlers to decode implemented commands; verify `PID_DEBUG` by raw serial
   capture because its current handler only reports `Not implemented yet!`.
6. Invoke raw `SendFrame()` with a 255-byte payload, confirm it returns
   `LibXR::ErrorCode::OK`, and capture one valid 261-byte UART frame.

Expected: all six checks pass. If hardware is unavailable, mark these six
items explicitly as pending in the handoff; do not represent format and build
results as hardware protocol verification. Remove any temporary debug call
site before Step 6 and confirm it is absent from both repository diffs.

- [ ] **Step 6: Audit repository boundaries and final commits**

Run:

```bash
git -C Modules/WsProtocol status --short --branch
git -C Modules/WsProtocol log -n 4 --oneline --decorate
git status --short --branch
git log -n 10 --oneline --decorate
```

Expected:

- the module repository is clean and contains the receive, transmit, and README
  commits on top of `a31927f`;
- the root repository contains separate navigation configuration/test cleanup,
  USART6 source configuration, and generated hardware-map commits;
- the pre-existing root `AGENTS.md` modification remains unstaged and
  untouched;
- no generated files other than the intentional `User/app_main.cpp` capacity
  sync, build products, vendor files, or unrelated module files are staged or
  committed.

Do not push either repository as part of plan execution without separate user
authorization. Before another developer consumes the root configuration, the
module commits must be published to `GUET-PLDX/WsProtocol` or otherwise made
available through the module registry.
