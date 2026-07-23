#!/usr/bin/env bash

set -euo pipefail

header=${1:-HostData.hpp}

if [[ ! -f "$header" ]]; then
  printf 'missing header: %s\n' "$header" >&2
  exit 1
fi

fail() {
  printf 'missing: %s\n' "$1" >&2
  exit 1
}

require_file_text() {
  local text=$1
  local description=$2
  grep -Fq "$text" "$header" || fail "$description"
}

require_file_pattern() {
  local pattern=$1
  local description=$2
  grep -Pzq "$pattern" "$header" || fail "$description"
}

forbid_file_text() {
  local text=$1
  local description=$2
  if grep -Fq "$text" "$header"; then
    fail "$description"
  fi
}

extract_block() {
  local start_pattern=$1
  awk -v start_pattern="$start_pattern" '
    { sub(/\r$/, "") }
    $0 ~ start_pattern { active = 1 }
    active {
      print
      opens = gsub(/{/, "{")
      closes = gsub(/}/, "}")
      depth += opens - closes
      if (opens > 0) {
        saw_open = 1
      }
      if (saw_open && depth == 0) {
        exit
      }
    }
  ' "$header"
}

require_block_text() {
  local block=$1
  local text=$2
  local description=$3
  grep -Fq "$text" <<<"$block" || fail "$description"
}

require_block_pattern() {
  local block=$1
  local pattern=$2
  local description=$3
  grep -Pzq "$pattern" <<<"$block" || fail "$description"
}

owner_thread=$(extract_block 'static void ThreadFunc\(HostData\* host_data\)')
[[ -n "$owner_thread" ]] || fail 'HostData owner thread'

apply_gimbal=$(extract_block 'void ApplyGimbal\(const HostGimbalTarget&')
[[ -n "$apply_gimbal" ]] || fail 'ApplyGimbal owner helper'

freshness_changed=$(extract_block 'bool FreshnessChanged\(LibXR::MillisecondTimestamp')
[[ -n "$freshness_changed" ]] || fail 'FreshnessChanged owner helper'

is_fresh=$(extract_block 'static bool IsFresh\(bool received,')
[[ -n "$is_fresh" ]] || fail 'received-aware IsFresh helper'

build_host_cmd=$(extract_block 'CMD::Data BuildHostCMD\(LibXR::MillisecondTimestamp')
[[ -n "$build_host_cmd" ]] || fail 'BuildHostCMD helper'

on_monitor=$(extract_block 'void OnMonitor\(\) override')
[[ -n "$on_monitor" ]] || fail 'OnMonitor override'

require_file_text 'task_stack_depth: 1024' 'task_stack_depth manifest argument'
require_file_text \
  'thread_priority: LibXR::Thread::Priority::MEDIUM' \
  'thread_priority manifest argument'
require_file_text \
  'uint32_t task_stack_depth,' \
  'task_stack_depth constructor argument'
require_file_pattern \
  'LibXR::Thread::Priority thread_priority\s*=\s*LibXR::Thread::Priority::MEDIUM' \
  'thread_priority constructor argument'
require_file_text \
  'thread_.Create(this, ThreadFunc, "HostDataThread", task_stack_depth,' \
  'HostData thread creation'
require_file_text 'LibXR::Thread thread_;' 'HostData thread member'

forbid_file_text 'RegisterCallback' 'callback registration must be removed'
forbid_file_text 'HostCMD(bool' 'callback-era HostCMD must be removed'
forbid_file_text 'LibXR::Mutex' 'HostData must not lock shared state'

require_block_text "$owner_thread" \
  'LibXR::Topic::ASyncSubscriber<HostGimbalTarget> gimbal_sub(' \
  'gimbal asynchronous subscriber'
require_block_text "$owner_thread" \
  'LibXR::Topic::ASyncSubscriber<HostChassisTarget> chassis_sub(' \
  'chassis asynchronous subscriber'
require_block_text "$owner_thread" \
  'LibXR::Topic::ASyncSubscriber<LauncherCMD> fire_sub(' \
  'fire asynchronous subscriber'

for subscriber in gimbal_sub chassis_sub fire_sub; do
  start_count=$(grep -Fc "${subscriber}.StartWaiting();" <<<"$owner_thread")
  [[ "$start_count" -eq 2 ]] || fail "${subscriber} initial arm and rearm"
done

require_block_pattern "$owner_thread" \
  'gimbal_sub\.StartWaiting\(\);\s*chassis_sub\.StartWaiting\(\);\s*fire_sub\.StartWaiting\(\);\s*LibXR::MillisecondTimestamp last_time\s*=\s*LibXR::Timebase::GetMilliseconds\(\);\s*while \(true\)' \
  'all subscribers armed before the owner loop'
require_block_pattern "$owner_thread" \
  'const auto DATA\s*=\s*gimbal_sub\.GetData\(\);\s*host_data->ApplyGimbal\(DATA, NOW\);\s*gimbal_sub\.StartWaiting\(\);' \
  'gimbal subscriber rearm after consumption'
require_block_pattern "$owner_thread" \
  'host_data->host_chassis_data_\s*=\s*chassis_sub\.GetData\(\);\s*host_data->last_chassis_time_\s*=\s*NOW;\s*host_data->chassis_received_\s*=\s*true;\s*chassis_sub\.StartWaiting\(\);' \
  'chassis subscriber rearm after consumption'
require_block_pattern "$owner_thread" \
  'host_data->host_fire_notify_\s*=\s*fire_sub\.GetData\(\);\s*host_data->last_fire_time_\s*=\s*NOW;\s*host_data->fire_received_\s*=\s*true;\s*fire_sub\.StartWaiting\(\);' \
  'fire subscriber rearm after consumption'

require_block_text "$owner_thread" 'bool updated = false;' \
  'owner-loop update flag'
require_block_text "$owner_thread" \
  'const bool FRESHNESS_CHANGED = host_data->FreshnessChanged(NOW);' \
  'freshness transition sampled every owner-loop iteration'
require_block_pattern "$owner_thread" \
  'if \(updated \|\| FRESHNESS_CHANGED\) \{\s*host_data->cmd_->FeedAI\(host_data->BuildHostCMD\(NOW\)\);\s*\}' \
  'FeedAI guarded by updates or freshness transitions'
require_block_text "$owner_thread" \
  'host_data->thread_.SleepUntil(last_time, 5);' \
  '5 ms owner-loop period'

feed_count=$(grep -Fc 'FeedAI(' "$header")
[[ "$feed_count" -eq 1 ]] || fail 'single owner-thread FeedAI call'

require_block_text "$apply_gimbal" \
  'host_euler_ = LibXR::EulerAngle<float>(data.rol, data.pit, data.yaw);' \
  'gimbal Euler copy'
require_block_text "$apply_gimbal" \
  'host_gyro_ =' \
  'gimbal angular velocity copy'
require_block_text "$apply_gimbal" \
  'host_accl_ =' \
  'gimbal acceleration copy'
require_block_text "$apply_gimbal" 'last_gimbal_time_ = now;' \
  'gimbal timestamp update'
require_block_text "$apply_gimbal" 'gimbal_received_ = true;' \
  'gimbal received-state update'

for state in chassis gimbal fire; do
  require_file_text "bool ${state}_received_ = false;" \
    "${state} received-state member"
  require_block_text "$freshness_changed" \
    "this->IsFresh(${state}_received_, last_${state}_time_, now)" \
    "${state} freshness sample"
  require_block_text "$freshness_changed" \
    "${state}_fresh_ = ${state^^}_FRESH;" \
    "${state} freshness state update"
  require_block_text "$build_host_cmd" \
    "this->IsFresh(${state}_received_, last_${state}_time_, now)" \
    "${state} BuildHostCMD freshness guard"
done
require_block_pattern "$freshness_changed" \
  'const bool CHANGED\s*=\s*CHASSIS_FRESH != chassis_fresh_\s*\|\|\s*GIMBAL_FRESH != gimbal_fresh_\s*\|\|\s*FIRE_FRESH != fire_fresh_;' \
  'freshness edge must compare all current and previous states'
require_block_text "$freshness_changed" 'return CHANGED;' \
  'freshness transition result'

require_block_pattern "$is_fresh" \
  'return received\s*&&\s*\(now - last_time\)\.ToMillisecond\(\) <=\s*HOST_DATA_TIMEOUT_MS;' \
  'received-aware wrap-safe freshness duration'
forbid_file_text 'static_cast<uint32_t>(last_time) != 0U' \
  'timestamp zero must not mean never received'

writer_blocks="${owner_thread}"$'\n'"${apply_gimbal}"$'\n'"${freshness_changed}"
for member in host_chassis_data_ host_fire_notify_ host_euler_ host_gyro_ \
  host_accl_ last_chassis_time_ last_gimbal_time_ last_fire_time_ \
  chassis_received_ gimbal_received_ fire_received_ chassis_fresh_ \
  gimbal_fresh_ fire_fresh_; do
  all_assignment_count=$(grep -Po \
    "(?:host_data->)?${member}[[:space:]]*=" "$header" | wc -l)
  owner_assignment_count=$(grep -Po \
    "(?:host_data->)?${member}[[:space:]]*=" <<<"$writer_blocks" | wc -l)
  [[ "$owner_assignment_count" -eq 1 ]] || \
    fail "single owner writer: ${member}"

  expected_assignment_count=2
  if [[ "$member" == host_euler_ ]]; then
    expected_assignment_count=1
  fi
  [[ "$all_assignment_count" -eq "$expected_assignment_count" ]] || \
    fail "owner-only global writer: ${member}"
done

compact_monitor=$(tr -d '[:space:]' <<<"$on_monitor")
[[ "$compact_monitor" == 'voidOnMonitor()override{}' ]] || \
  fail 'OnMonitor must be empty'

printf 'PASS: HostData single-owner regression\n'
