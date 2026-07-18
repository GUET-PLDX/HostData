$ErrorActionPreference = 'Stop'

function Assert-Contains {
  param([string]$Path, [string]$Pattern, [string]$Message)
  if (-not (Select-String -Path $Path -Pattern $Pattern -Quiet -CaseSensitive -SimpleMatch)) {
    throw $Message
  }
}

function Assert-NotContains {
  param([string]$Path, [string]$Pattern, [string]$Message)
  if (Select-String -Path $Path -Pattern $Pattern -Quiet -CaseSensitive -SimpleMatch) {
    throw $Message
  }
}

function Assert-StructFieldOrder {
  param(
      [string]$Path,
      [string]$StructName,
      [string[]]$Fields,
      [string]$Message
  )

  $lines = Get-Content -LiteralPath $Path
  $struct_start = -1
  for ($index = 0; $index -lt $lines.Count; $index++) {
    if ($lines[$index] -match "^\s*struct\b.*\b$([regex]::Escape($StructName))\s*\{") {
      $struct_start = $index
      break
    }
  }
  if ($struct_start -lt 0) {
    throw "$Message $StructName declaration is missing."
  }

  $struct_end = $struct_start + 1
  while ($struct_end -lt $lines.Count -and $lines[$struct_end] -notmatch '^\s*};') {
    $struct_end++
  }
  if ($struct_end -ge $lines.Count) {
    throw "$Message $StructName declaration is not closed."
  }

  $previous_field_line = $struct_start
  foreach ($field in $Fields) {
    $field_line = -1
    for ($index = $previous_field_line + 1; $index -lt $struct_end; $index++) {
      if ($lines[$index].Contains($field)) {
        $field_line = $index
        break
      }
    }
    if ($field_line -lt 0) {
      throw "$Message '$field' is missing or out of order in $StructName."
    }
    $previous_field_line = $field_line
  }
}

function Assert-RefereeFreshnessGuard {
  param([string]$Path, [string]$Message)

  $content = Get-Content -LiteralPath $Path -Raw
  $guard_pattern = 'if\s*\(\s*referee_received_\s*&&\s*now_ms\s*-\s*last_referee_rx_time_ms_\s*<=\s*REFEREE_RX_TIMEOUT_MS\s*\)\s*\{'
  $guard = [regex]::Match($content, $guard_pattern)
  if (-not $guard.Success) {
    throw "$Message Fresh referee-data guard is missing."
  }

  $open_brace = $guard.Index + $guard.Length - 1
  $depth = 0
  $close_brace = -1
  for ($index = $open_brace; $index -lt $content.Length; $index++) {
    if ($content[$index] -eq '{') {
      $depth++
    } elseif ($content[$index] -eq '}') {
      $depth--
      if ($depth -eq 0) {
        $close_brace = $index
        break
      }
    }
  }
  if ($close_brace -lt 0) {
    throw "$Message Fresh referee-data guard is not closed."
  }

  $guard_body = $content.Substring($open_brace, $close_brace - $open_brace + 1)
  $assignments = @(
      'command.referee_power_limit = referee_power_limit_;',
      'command.referee_energy_buffer = referee_energy_buffer_;'
  )
  foreach ($assignment in $assignments) {
    if (-not $guard_body.Contains($assignment) -or
        [regex]::Matches($content, [regex]::Escape($assignment)).Count -ne 1) {
      throw "$Message '$assignment' must occur only in the fresh referee-data guard."
    }
  }
}

$protocol = 'Modules/SuperPower/SuperPowerProtocol.hpp'
$module = 'Modules/SuperPower/SuperPower.hpp'

Assert-Contains $protocol 'FEEDBACK_ID = 0x051U' 'Feedback CAN ID must be 0x051.'
Assert-Contains $protocol 'COMMAND_ID = 0x061U' 'Command CAN ID must be 0x061.'
Assert-Contains $protocol 'ENABLE_DCDC_MASK = 0x01U' 'DCDC enable mask must set bit zero.'
Assert-Contains $protocol 'uint8_t error_code' 'Feedback must carry an error code.'
Assert-Contains $protocol 'float chassis_power' 'Feedback must carry float chassis power.'
Assert-Contains $protocol 'uint16_t chassis_power_limit' 'Feedback must carry chassis power limit.'
Assert-Contains $protocol 'uint8_t cap_energy' 'Feedback must carry capacitor energy.'
Assert-Contains $protocol 'uint8_t flags' 'Command must carry command flags.'
Assert-Contains $protocol 'uint16_t referee_power_limit' 'Command must carry referee power limit.'
Assert-Contains $protocol 'uint16_t referee_energy_buffer' 'Command must carry referee buffer.'
Assert-Contains $protocol 'uint8_t reserved[3]' 'Command must reserve the final three bytes.'
Assert-StructFieldOrder $protocol 'FeedbackData' @(
    'uint8_t error_code',
    'float chassis_power',
    'uint16_t chassis_power_limit',
    'uint8_t cap_energy'
) 'Feedback field order must match the RM2024 wire format.'
Assert-StructFieldOrder $protocol 'CommandData' @(
    'uint8_t flags',
    'uint16_t referee_power_limit',
    'uint16_t referee_energy_buffer',
    'uint8_t reserved[3]'
) 'Command field order must match the RM2024 wire format.'
Assert-Contains $protocol 'sizeof(FeedbackData) == 8U' 'Feedback layout must be eight bytes.'
Assert-Contains $protocol 'sizeof(CommandData) == 8U' 'Command layout must be eight bytes.'
Assert-Contains $protocol 'std::array<uint8_t, sizeof(CommandData)> bytes{}' 'Command codec must zero-initialize all output bytes.'
Assert-Contains $protocol 'sizeof(command.reserved)' 'Command codec must exclude reserved bytes from its copy.'
Assert-Contains $protocol 'std::memcpy(bytes.data(), &command, COMMAND_PAYLOAD_SIZE)' 'Command codec must copy only active command bytes.'
Assert-Contains $module 'COMMAND_PERIOD_MS = 5U' 'Command period must be 5 ms.'
Assert-Contains $module 'STATUS_RX_TIMEOUT_MS = 100U' 'Feedback timeout must be 100 ms.'
Assert-Contains $module 'REFEREE_RX_TIMEOUT_MS = 1000U' 'Referee-data timeout must be 1000 ms.'
Assert-Contains $module 'Timer::CreateTask' 'CAN output must run from a timer.'
Assert-Contains $module 'referee_energy_buffer' 'Module must forward referee buffer.'
Assert-Contains $module 'chassis_pack.rs.chassis_power_limit' 'Chassis callback must forward the referee power limit.'
Assert-Contains $module 'chassis_pack.power_buffer' 'Chassis callback must forward the referee energy buffer.'
Assert-Contains $module 'referee_received_ = true;' 'Draining referee data must mark referee data as received.'
Assert-Contains $module 'bool referee_received_ = false;' 'Referee data presence must use an explicit receive flag.'
Assert-NotContains $module 'last_referee_rx_time_ms_ != 0U' 'Referee data presence must not rely on a nonzero timestamp.'
Assert-Contains $module 'SuperPowerProtocol::CommandData command{}' 'Transmit path must value-initialize the command packet.'
Assert-Contains $module 'command.flags = SuperPowerProtocol::ENABLE_DCDC_MASK' 'Command must enable DCDC.'
Assert-Contains $module 'LibXR::Memory::FastCopy(tx_pack.data, &command, sizeof(command))' 'Transmit path must copy the complete initialized command packet.'
Assert-RefereeFreshnessGuard $module 'Stale referee data must leave both command values zero.'
Assert-NotContains $module 'command.reserved' 'Transmit path must not write reserved command bytes.'
Assert-NotContains $module 'SAME_FRAME_OFFLINE_COUNT' 'Identical frames must remain online.'
Assert-NotContains $module 'POWER_ENCODE_OFFSET' 'RM2024 power is a float, not offset binary.'

Write-Output 'PASS: RM2024 SuperPower protocol static checks'
