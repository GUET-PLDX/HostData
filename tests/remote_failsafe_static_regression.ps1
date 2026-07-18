$ErrorActionPreference = 'Stop'

function Assert-Contains {
  param(
    [string]$Path,
    [string]$Pattern,
    [string]$Message
  )

  if (-not (Select-String -Path $Path -Pattern $Pattern -Quiet -CaseSensitive)) {
    throw $Message
  }
}

$cmdHeader = 'Modules/CMD/CMD.hpp'
$refereeHeader = 'Modules/Referee/Referee.hpp'

Assert-Contains $cmdHeader 'PublishSafeStopCommands' 'CMD must expose PublishSafeStopCommands.'
Assert-Contains $cmdHeader '遥控失联时优先失能|禁止自动控制继续输出' 'CMD must document the remote-offline safe-stop priority.'
Assert-Contains $cmdHeader 'if \(!rc_data\.chassis_online\)' 'CMD must branch before OP/AUTO output when chassis remote is offline.'
Assert-Contains $cmdHeader 'this->PublishSafeStopCommands\(\);' 'CMD must publish safe-stop commands when remote is offline.'
Assert-Contains $cmdHeader 'return;' 'CMD must return immediately after remote-offline safe stop.'
Assert-Contains $cmdHeader 'gimbal_data_tp_\.Publish' 'PublishSafeStopCommands must publish a safe gimbal command.'
Assert-Contains $cmdHeader 'chassis_data_tp_\.Publish' 'PublishSafeStopCommands must publish a safe chassis command.'
Assert-Contains $cmdHeader 'fire_data_tp_\.Publish' 'PublishSafeStopCommands must publish a safe launcher command.'

Assert-Contains $refereeHeader 'VIDEO_LINK_REMOTE_TIMEOUT_MS = 100' 'Referee video-link remote timeout must stay 100 ms.'
Assert-Contains $refereeHeader 'REFEREE_RX_TIMEOUT_MS = 50' 'Referee UART read timeout must stay short enough for remote failsafe polling while allowing long referee frames.'
Assert-Contains $refereeHeader 'op_\(sem_, REFEREE_RX_TIMEOUT_MS\)' 'Referee UART read operation must use the short failsafe-aware timeout.'
Assert-Contains $refereeHeader 'video_link_remote_last_time_' 'Referee must track the last video-link remote update time.'
Assert-Contains $refereeHeader 'video_link_remote_online_' 'Referee must latch video-link remote online state.'
Assert-Contains $refereeHeader 'ref->CheckVideoLinkRemoteOffline\(\);' 'Referee thread must poll video-link remote offline state every loop.'
Assert-Contains $refereeHeader 'void OnMonitor\(\) override \{ CheckVideoLinkRemoteOffline\(\); \}' 'OnMonitor must remain a fallback for video-link remote offline checks.'
Assert-Contains $refereeHeader 'this->CheckVideoLinkRemoteOffline\(\);' 'FindHeader must check video-link remote offline state while waiting for referee bytes.'
Assert-Contains $refereeHeader 'cmd_ == nullptr \|\| !video_link_remote_online_' 'Video-link offline helper must skip when CMD is absent or video-link remote is offline.'
Assert-Contains $refereeHeader 'NOW - video_link_remote_last_time_ <= VIDEO_LINK_REMOTE_TIMEOUT_MS' 'Video-link offline helper must compare elapsed time against the timeout.'
Assert-Contains $refereeHeader 'cmd_data\.chassis_online = false' 'Video-link timeout path must mark chassis offline.'
Assert-Contains $refereeHeader 'cmd_data\.gimbal_online = false' 'Video-link timeout path must mark gimbal offline.'
Assert-Contains $refereeHeader 'cmd_->FeedRC\(cmd_data\)' 'Video-link timeout path must feed the offline zero command to CMD.'
Assert-Contains $refereeHeader 'video_link_remote_online_ = false' 'Video-link timeout path must clear the online latch.'

Write-Output 'PASS: remote failsafe static checks'
