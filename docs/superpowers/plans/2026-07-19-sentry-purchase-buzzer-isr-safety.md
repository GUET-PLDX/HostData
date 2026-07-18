# Sentry Purchase and Buzzer ISR Safety Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve `uint16_t` local sentry purchase increments through checked 11-bit accumulation, separate remote-purchase request counting, and prevent the buzzer fatal callback from blocking in ISR context.

**Architecture:** `SentryProtocol` keeps Topic payload semantics at the application boundary and calls explicit checked mutation APIs on `Referee`; `Referee` owns the locked cumulative wire fields and rejects unrepresentable updates. `BuzzerAlarm` exits before all PWM and sleep work when `in_isr` is true. Root-level static regression scripts protect these cross-repository contracts, followed by both firmware builds.

**Tech Stack:** C++17, LibXR `ErrorCode` and `Mutex`, xrobot module headers, Bash/Python static regression scripts, clang-format 21.1.8, CMake/STM32 cross-build via `tools/build.sh`.

## Global Constraints

- Do not modify BMI088 or CameraSync.
- Do not modify `Drivers/` or `Middlewares/`.
- Preserve Topic names and payload widths: local purchase remains `uint16_t`; remote trigger remains `uint8_t`.
- All constants use `UPPER_CASE`; methods use `CamelCase`.
- Format only changed files under `Modules/` with clang-format 21.1.8.
- Preserve all pre-existing dirty-worktree changes.
- `Modules/Referee`, `Modules/SentryProtocol`, and `Modules/BuzzerAlarm` are independent Git repositories.
- Never run `git add` on a dirty module header as a whole. Stage only the incremental hunk introduced by this plan and inspect `git diff --cached` before committing.
- Do not edit generated `User/xrobot_main.hpp` or commit `build/` artifacts.

---

### Task 1: Add failing sentry purchase contract regression

**Files:**
- Create: `tests/sentry_purchase_static_regression.sh`
- Inspect: `Modules/Referee/Referee.hpp:878-887,1052-1081`
- Inspect: `Modules/SentryProtocol/SentryProtocol.hpp:115-129`

**Interfaces:**
- Consumes: current `SetNeedBullet(uint8_t)` and `SetBulletRemote(uint8_t)` implementation.
- Produces: a root-level executable regression test for `AddNeedBullet(uint16_t)` and `RequestRemoteBulletExchange()`.

- [ ] **Step 1: Write the failing static regression test**

Create `tests/sentry_purchase_static_regression.sh` with this complete content:

```bash
#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

python3 - "${ROOT_DIR}/Modules/Referee/Referee.hpp" \
  "${ROOT_DIR}/Modules/SentryProtocol/SentryProtocol.hpp" <<'PY'
import re
import sys

referee = open(sys.argv[1], encoding="utf-8").read()
sentry = open(sys.argv[2], encoding="utf-8").read()


def body_after(source, marker):
    start = source.find(marker)
    if start < 0:
        raise SystemExit(f"FAIL: missing {marker}")
    brace = source.find("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1:index]
    raise SystemExit(f"FAIL: unterminated body for {marker}")


def need(source, pattern, label):
    if re.search(pattern, source, re.DOTALL) is None:
        raise SystemExit(f"FAIL: missing {label}")


need(referee, r"MAX_BUY_BULLET_NUM\s*=\s*\(1U\s*<<\s*11U\)\s*-\s*1U",
     "11-bit local purchase limit")
need(referee, r"MAX_REMOTE_BUY_BULLET_TIMES\s*=\s*\(1U\s*<<\s*4U\)\s*-\s*1U",
     "4-bit remote request limit")

local_body = body_after(
    referee, "LibXR::ErrorCode AddNeedBullet(uint16_t bullet_delta)")
need(local_body, r"bullet_delta\s*==\s*0U.*?ErrorCode::ARG_ERR",
     "zero-delta rejection")
need(local_body, r"UPDATED_BULLET_NUM\s*>\s*MAX_BUY_BULLET_NUM.*?"
                 r"ErrorCode::OUT_OF_RANGE",
     "11-bit overflow rejection")
need(local_body, r"buy_bullet_num\s*=\s*UPDATED_BULLET_NUM",
     "checked cumulative assignment")
need(local_body, r"ErrorCode::OK", "successful local purchase result")

remote_body = body_after(
    referee, "LibXR::ErrorCode RequestRemoteBulletExchange()")
need(remote_body, r"remote_buy_bullet_times\s*>=\s*"
                  r"MAX_REMOTE_BUY_BULLET_TIMES.*?ErrorCode::OUT_OF_RANGE",
     "remote counter overflow rejection")
need(remote_body, r"remote_buy_bullet_times\s*\+\+",
     "single remote request increment")
if "buy_bullet_num" in remote_body:
    raise SystemExit("FAIL: remote request still modifies local purchase total")

local_handler = body_after(sentry, "void OnBuyBulletTopic(")
need(local_handler, r"uint16_t\s+buy_bullet_num", "uint16_t Topic payload")
need(local_handler, r"AddNeedBullet\(buy_bullet_num\)\s*==\s*"
                    r"LibXR::ErrorCode::OK.*?SendSentryPack",
     "send only after accepted local increment")
if "static_cast<uint8_t>" in local_handler:
    raise SystemExit("FAIL: local purchase still narrows to uint8_t")

remote_handler = body_after(sentry, "void OnRemoteBuyBulletTopic(")
need(remote_handler, r"uint8_t\s+remote_buy_bullet_request",
     "uint8_t remote trigger")
need(remote_handler, r"remote_buy_bullet_request\s*!=\s*0U",
     "zero remote trigger ignored")
need(remote_handler, r"RequestRemoteBulletExchange\(\)\s*==\s*"
                     r"LibXR::ErrorCode::OK.*?SendSentryPack",
     "send only after accepted remote request")

print("PASS: sentry purchase contract regression")
PY
```

- [ ] **Step 2: Mark the test executable and verify it fails for the old API**

Run:

```bash
chmod +x tests/sentry_purchase_static_regression.sh
bash tests/sentry_purchase_static_regression.sh
```

Expected: nonzero exit with `FAIL: missing 11-bit local purchase limit` or `FAIL: missing LibXR::ErrorCode AddNeedBullet(uint16_t bullet_delta)`.

---

### Task 2: Implement checked sentry purchase mutations

**Files:**
- Modify: `Modules/Referee/Referee.hpp:1052-1081`
- Modify: `Modules/SentryProtocol/SentryProtocol.hpp:115-129`
- Test: `tests/sentry_purchase_static_regression.sh`

**Interfaces:**
- Consumes: `uint16_t` local increment and nonzero `uint8_t` remote trigger from Task 1.
- Produces: `LibXR::ErrorCode AddNeedBullet(uint16_t bullet_delta)` and `LibXR::ErrorCode RequestRemoteBulletExchange()`.

- [ ] **Step 1: Snapshot dirty module headers before editing**

Run:

```bash
cp Modules/Referee/Referee.hpp /tmp/pldx-referee-before.hpp
cp Modules/SentryProtocol/SentryProtocol.hpp /tmp/pldx-sentry-protocol-before.hpp
```

Expected: both snapshots exist; no Git state changes.

- [ ] **Step 2: Add checked counters and explicit APIs in Referee**

Add these constants near the other public protocol constants in `Referee`:

```cpp
static constexpr uint32_t MAX_BUY_BULLET_NUM = (1U << 11U) - 1U;
static constexpr uint32_t MAX_REMOTE_BUY_BULLET_TIMES = (1U << 4U) - 1U;
```

Replace `SetNeedBullet` and `SetBulletRemote` with:

```cpp
LibXR::ErrorCode AddNeedBullet(uint16_t bullet_delta) {
  if (bullet_delta == 0U) {
    return LibXR::ErrorCode::ARG_ERR;
  }

  LibXR::Mutex::LockGuard lock(tx_data_mutex_);
  const uint32_t UPDATED_BULLET_NUM =
      this->data_.sentry_dec_data.buy_bullet_num + bullet_delta;
  if (UPDATED_BULLET_NUM > MAX_BUY_BULLET_NUM) {
    return LibXR::ErrorCode::OUT_OF_RANGE;
  }

  this->data_.sentry_dec_data.buy_bullet_num = UPDATED_BULLET_NUM;
  return LibXR::ErrorCode::OK;
}

LibXR::ErrorCode RequestRemoteBulletExchange() {
  LibXR::Mutex::LockGuard lock(tx_data_mutex_);
  if (this->data_.sentry_dec_data.remote_buy_bullet_times >=
      MAX_REMOTE_BUY_BULLET_TIMES) {
    return LibXR::ErrorCode::OUT_OF_RANGE;
  }

  this->data_.sentry_dec_data.remote_buy_bullet_times++;
  return LibXR::ErrorCode::OK;
}
```

Update the Doxygen text so `AddNeedBullet` is documented as one local increment and `RequestRemoteBulletExchange` as one remote request. Do not change `SentryDecisionData` field widths.

- [ ] **Step 3: Preserve Topic widths and gate packet sending in SentryProtocol**

Replace the two handlers with:

```cpp
void OnBuyBulletTopic(const LibXR::ConstRawData& raw_data) {
  uint16_t buy_bullet_num = 0;
  if (ReadTopicData(raw_data, buy_bullet_num) && referee_ != nullptr &&
      referee_->AddNeedBullet(buy_bullet_num) == LibXR::ErrorCode::OK) {
    referee_->SendSentryPack();
  }
}

void OnRemoteBuyBulletTopic(const LibXR::ConstRawData& raw_data) {
  uint8_t remote_buy_bullet_request = 0;
  if (ReadTopicData(raw_data, remote_buy_bullet_request) &&
      remote_buy_bullet_request != 0U && referee_ != nullptr &&
      referee_->RequestRemoteBulletExchange() == LibXR::ErrorCode::OK) {
    referee_->SendSentryPack();
  }
}
```

- [ ] **Step 4: Format only the two changed module headers**

Run:

Run `nl -ba` first to locate the new constants, methods, and handlers, then format only
those line spans (the shown ranges include a small context margin):

```bash
nl -ba Modules/Referee/Referee.hpp | rg "MAX_BUY_BULLET_NUM|AddNeedBullet|RequestRemoteBulletExchange"
nl -ba Modules/SentryProtocol/SentryProtocol.hpp | rg "OnBuyBulletTopic|OnRemoteBuyBulletTopic"
clang-format -i --lines=50:70 --lines=1050:1105 Modules/Referee/Referee.hpp
clang-format -i --lines=110:135 Modules/SentryProtocol/SentryProtocol.hpp
clang-format --version
```

Expected: version contains `21.1.8`; no unrelated module file changes.

- [ ] **Step 5: Run the sentry regression and chassis compile**

Run:

```bash
bash tests/sentry_purchase_static_regression.sh
tools/build.sh --skip-format -c User/RobotConfig/sentry_chassis.yaml -b build/sentry_chassis
```

Expected: regression prints `PASS: sentry purchase contract regression`; chassis build exits 0 with no `-Werror` diagnostics.

- [ ] **Step 6: Stage and commit only the incremental Referee hunk**

Generate a zero-context incremental patch and apply it to the Referee index only. The
`diff` command is expected to exit 1 because differences are present while still
writing the patch file:

```bash
diff -U0 --label a/Referee.hpp --label b/Referee.hpp /tmp/pldx-referee-before.hpp Modules/Referee/Referee.hpp > /tmp/pldx-referee-fix.patch
git -C Modules/Referee apply --cached --unidiff-zero /tmp/pldx-referee-fix.patch
git -C Modules/Referee diff --cached --check
git -C Modules/Referee diff --cached
git -C Modules/Referee commit -m "fix: validate sentry purchase counters"
```

Expected staged diff: only the two constants, two purchase methods, and their comments. It must not include the pre-existing 2026 protocol migration changes.

- [ ] **Step 7: Stage and commit only the incremental SentryProtocol hunk**

Generate and stage the SentryProtocol incremental patch the same way:

```bash
diff -U0 --label a/SentryProtocol.hpp --label b/SentryProtocol.hpp /tmp/pldx-sentry-protocol-before.hpp Modules/SentryProtocol/SentryProtocol.hpp > /tmp/pldx-sentry-protocol-fix.patch
git -C Modules/SentryProtocol apply --cached --unidiff-zero /tmp/pldx-sentry-protocol-fix.patch
git -C Modules/SentryProtocol diff --cached --check
git -C Modules/SentryProtocol diff --cached
git -C Modules/SentryProtocol commit -m "fix: preserve sentry purchase increments"
```

Expected staged diff: only the two purchase handler bodies. It must not include existing manifest or `ConstRawData` migration changes.

---

### Task 3: Add failing Buzzer ISR regression

**Files:**
- Create: `tests/buzzer_isr_static_regression.sh`
- Inspect: `Modules/BuzzerAlarm/BuzzerAlarm.hpp:33-45`

**Interfaces:**
- Consumes: fatal callback parameter `bool in_isr`.
- Produces: an executable regression that requires an early ISR return before `Play`.

- [ ] **Step 1: Write the failing static regression test**

Create `tests/buzzer_isr_static_regression.sh` with:

```bash
#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

python3 - "${ROOT_DIR}/Modules/BuzzerAlarm/BuzzerAlarm.hpp" <<'PY'
import re
import sys

source = open(sys.argv[1], encoding="utf-8").read()
start = source.find("[](bool in_isr, BuzzerAlarm* alarm")
if start < 0:
    raise SystemExit("FAIL: missing fatal callback")
brace = source.find("{", start)
depth = 0
callback = None
for index in range(brace, len(source)):
    if source[index] == "{":
        depth += 1
    elif source[index] == "}":
        depth -= 1
        if depth == 0:
            callback = source[brace + 1:index]
            break
if callback is None:
    raise SystemExit("FAIL: unterminated fatal callback")

guard = re.search(r"if\s*\(in_isr\)\s*\{\s*return;\s*\}", callback)
if guard is None:
    raise SystemExit("FAIL: missing ISR early return")
play = callback.find("alarm->Play")
if play < 0 or guard.start() > play:
    raise SystemExit("FAIL: ISR guard does not precede Play")
if "Thread::Sleep" in callback[:guard.end()]:
    raise SystemExit("FAIL: sleep remains before ISR return")

print("PASS: BuzzerAlarm ISR regression")
PY
```

- [ ] **Step 2: Mark executable and verify red state**

Run:

```bash
chmod +x tests/buzzer_isr_static_regression.sh
bash tests/buzzer_isr_static_regression.sh
```

Expected: nonzero exit with `FAIL: missing ISR early return`.

---

### Task 4: Make the Buzzer fatal callback ISR-safe

**Files:**
- Modify: `Modules/BuzzerAlarm/BuzzerAlarm.hpp:33-45`
- Test: `tests/buzzer_isr_static_regression.sh`

**Interfaces:**
- Consumes: `in_isr` supplied by `LibXR::Assert::FatalCallback`.
- Produces: no PWM or blocking work for `in_isr == true`; unchanged normal-context alarm behavior.

- [ ] **Step 1: Snapshot the dirty Buzzer header**

Run:

```bash
cp Modules/BuzzerAlarm/BuzzerAlarm.hpp /tmp/pldx-buzzer-before.hpp
```

- [ ] **Step 2: Add the early ISR return**

Insert the guard after the two `UNUSED` calls and before `alarm->Play`:

```cpp
if (in_isr) {
  return;
}

alarm->Play(alarm->alarm_freq_, alarm->alarm_duration_);
LibXR::Thread::Sleep(alarm->alarm_delay_);
```

Remove the obsolete `if (!in_isr)` wrapper. Do not change `Play`, `PlayNote`, startup notes, or LibXR.

- [ ] **Step 3: Format and run the focused regression**

Run:

```bash
nl -ba Modules/BuzzerAlarm/BuzzerAlarm.hpp | rg "in_isr|alarm->Play"
clang-format -i --lines=30:50 Modules/BuzzerAlarm/BuzzerAlarm.hpp
bash tests/buzzer_isr_static_regression.sh
```

Expected: `PASS: BuzzerAlarm ISR regression`.

- [ ] **Step 4: Compile both deployed Buzzer configurations**

Run:

```bash
tools/build.sh --skip-format -c User/RobotConfig/sentry_gimbal.yaml -b build/sentry_gimbal
tools/build.sh --skip-format -c User/RobotConfig/sentry_chassis.yaml -b build/sentry_chassis
```

Expected: both builds exit 0 with no `-Werror` diagnostics.

- [ ] **Step 5: Stage and commit only the incremental Buzzer hunk**

Generate and stage only the zero-context incremental Buzzer patch:

```bash
diff -U0 --label a/BuzzerAlarm.hpp --label b/BuzzerAlarm.hpp /tmp/pldx-buzzer-before.hpp Modules/BuzzerAlarm/BuzzerAlarm.hpp > /tmp/pldx-buzzer-fix.patch
git -C Modules/BuzzerAlarm apply --cached --unidiff-zero /tmp/pldx-buzzer-fix.patch
git -C Modules/BuzzerAlarm diff --cached --check
git -C Modules/BuzzerAlarm diff --cached
git -C Modules/BuzzerAlarm commit -m "fix: avoid blocking buzzer callback in ISR"
```

Expected staged diff: only the early return and removal of the now-redundant negative condition.

---

### Task 5: Run final verification and commit root artifacts

**Files:**
- Create: `tests/sentry_purchase_static_regression.sh`
- Create: `tests/buzzer_isr_static_regression.sh`
- Create: `docs/superpowers/plans/2026-07-19-sentry-purchase-buzzer-isr-safety.md`
- Verify: `Modules/Referee/Referee.hpp`
- Verify: `Modules/SentryProtocol/SentryProtocol.hpp`
- Verify: `Modules/BuzzerAlarm/BuzzerAlarm.hpp`

**Interfaces:**
- Consumes: all production changes from Tasks 2 and 4.
- Produces: reproducible root tests, clean focused diffs, and final build evidence.

- [ ] **Step 1: Run focused and adjacent referee regressions**

Run:

```bash
bash tests/sentry_purchase_static_regression.sh
bash tests/buzzer_isr_static_regression.sh
bash tests/referee_2026_protocol_static_regression.sh
bash tests/referee_chassis_freshness_static_regression.sh
bash tests/dualboard_referee_static_regression.sh
```

Expected: every script prints `PASS` and exits 0.

- [ ] **Step 2: Run format verification**

Run:

```bash
clang-format --dry-run --Werror --lines=50:70 --lines=1050:1105 Modules/Referee/Referee.hpp
clang-format --dry-run --Werror --lines=110:135 Modules/SentryProtocol/SentryProtocol.hpp
clang-format --dry-run --Werror --lines=30:50 Modules/BuzzerAlarm/BuzzerAlarm.hpp
tools/format_code.sh --check
```

Expected: targeted check exits 0. The repository-wide check should exit 0; if unrelated pre-existing module changes fail it, record the exact files without modifying them.

- [ ] **Step 3: Rebuild both robot configurations from the final tree**

Run:

```bash
tools/build.sh --skip-format -c User/RobotConfig/sentry_gimbal.yaml -b build/sentry_gimbal
tools/build.sh --skip-format -c User/RobotConfig/sentry_chassis.yaml -b build/sentry_chassis
```

Expected: both builds exit 0.

- [ ] **Step 4: Commit only the new root test and plan files**

Run:

```bash
git add docs/superpowers/plans/2026-07-19-sentry-purchase-buzzer-isr-safety.md tests/sentry_purchase_static_regression.sh tests/buzzer_isr_static_regression.sh
git diff --cached --check
git diff --cached
git commit -m "test: guard sentry purchase and buzzer ISR safety"
```

Expected staged diff: exactly the plan and two new scripts. Do not stage `AGENTS.md`, YAML files, existing tests, generated files, or build output.

- [ ] **Step 5: Record final repository state**

Run:

```bash
git status --short
git -C Modules/Referee status --short
git -C Modules/SentryProtocol status --short
git -C Modules/BuzzerAlarm status --short
git log -2 --oneline
git -C Modules/Referee log -1 --oneline
git -C Modules/SentryProtocol log -1 --oneline
git -C Modules/BuzzerAlarm log -1 --oneline
```

Expected: only the user's pre-existing unrelated modifications remain unstaged; the four new commits are visible in their respective repositories.
