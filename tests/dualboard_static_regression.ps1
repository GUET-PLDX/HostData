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

function Assert-NotContains {
  param(
    [string]$Path,
    [string]$Pattern,
    [string]$Message
  )

  if (Select-String -Path $Path -Pattern $Pattern -Quiet -CaseSensitive) {
    throw $Message
  }
}

$dualBoardHeader = 'Modules/DualBoard/DualBoard.hpp'
$motionStateHeader = 'Modules/Chassis/ChassisMotionState.hpp'
$dualBoardCMake = 'Modules/DualBoard/CMakeLists.txt'
$gimbalYaml = 'User/RobotConfig/sentry_gimbal.yaml'
$chassisYaml = 'User/RobotConfig/sentry_chassis.yaml'

Assert-Contains $dualBoardHeader 'enum class DualBoardRole' 'DualBoard role enum is missing.'
Assert-Contains $dualBoardHeader 'DualBoardRole::GIMBAL' 'DualBoard GIMBAL role branch is missing.'
Assert-Contains $dualBoardHeader 'DualBoardRole::CHASSIS' 'DualBoard CHASSIS role branch is missing.'
Assert-Contains $dualBoardHeader 'template <DualBoardRole ROLE, typename ChassisType = Omni>' 'DualBoard template signature is missing.'
Assert-Contains $dualBoardHeader 'LibXR::CAN::ClassicPack' 'DualBoard must use LibXR CAN classic frames.'
Assert-Contains $dualBoardHeader 'struct __attribute__\(\(packed\)\) ControlFrame' 'DualBoard must use a fixed chassis control frame.'
Assert-Contains $dualBoardHeader 'struct __attribute__\(\(packed\)\) AngleFrame' 'DualBoard must use a fixed gimbal angle frame.'
Assert-Contains $dualBoardHeader 'struct __attribute__\(\(packed\)\) AttitudeFrame' 'DualBoard must use a fixed attitude frame.'
Assert-Contains $dualBoardHeader 'struct __attribute__\(\(packed\)\) LauncherFeedbackFrame' 'DualBoard must use a compact launcher feedback frame.'
Assert-Contains $dualBoardHeader 'struct __attribute__\(\(packed\)\) MotionFrame' 'DualBoard MotionFrame is missing.'
Assert-Contains $dualBoardHeader 'CONTROL_PERIOD_MS = 10' 'DualBoard control frame period must be 10 ms.'
Assert-Contains $dualBoardHeader 'ANGLE_ID_OFFSET = 0x10U' 'DualBoard chassis tx_id 0x311 plus motion offset 0x10 must produce sentry CAN ID 0x321.'
Assert-Contains $dualBoardHeader 'LAUNCHER_FEEDBACK_PERIOD_MS = 20' 'DualBoard launcher feedback frame period must be 20 ms.'
Assert-Contains $dualBoardHeader 'static_assert\(sizeof\(ControlFrame\) == 8' 'DualBoard control frame must stay exactly one classic CAN frame.'
Assert-Contains $dualBoardHeader 'static_assert\(sizeof\(AngleFrame\) == 8' 'DualBoard angle frame must stay exactly one classic CAN frame.'
Assert-Contains $dualBoardHeader 'static_assert\(sizeof\(AttitudeFrame\) == 8' 'DualBoard attitude frame must stay exactly one classic CAN frame.'
Assert-Contains $dualBoardHeader 'static_assert\(sizeof\(LauncherFeedbackFrame\) == 8' 'DualBoard launcher feedback frame must stay exactly one classic CAN frame.'
Assert-Contains $dualBoardHeader 'static_assert\(sizeof\(MotionFrame\) == 8' 'DualBoard MotionFrame must stay exactly one classic CAN frame.'
Assert-Contains $dualBoardHeader 'GYRO_SCALE = 900\.0f' 'DualBoard gyro scale is missing.'
Assert-Contains $dualBoardHeader 'ChassisMotionState' 'DualBoard semantic chassis motion state is missing.'
Assert-Contains $motionStateHeader 'CHASSIS_MOTION_STATE_TOPIC_NAME' 'Motion state Topic name must be centralized.'
Assert-Contains $motionStateHeader 'CHASSIS_MOTION_STATE_TOPIC_MULTI_PUBLISHER' 'Motion state Topic attributes must be centralized.'
Assert-Contains $dualBoardHeader 'FindOrCreate<ChassisMotionState>' 'DualBoard must find or create the typed motion state Topic.'
Assert-NotContains $dualBoardHeader 'chassis_gyro_z_topic_' 'DualBoard must not retain the raw gyro Topic member.'
Assert-NotContains $dualBoardHeader 'chassis_alpha_z' 'DualBoard must not add chassis angular acceleration.'
$dualBoardLines = Get-Content -Path $dualBoardHeader
$motionFrameStartLine = (Select-String -Path $dualBoardHeader -Pattern '^  void HandleMotionFrame\(' -CaseSensitive).LineNumber
$controlFrameStartLine = (Select-String -Path $dualBoardHeader -Pattern '^  void HandleControlFrame\(' -CaseSensitive).LineNumber
if ($null -eq $motionFrameStartLine -or $null -eq $controlFrameStartLine -or $motionFrameStartLine -ge $controlFrameStartLine) {
  throw 'DualBoard MotionFrame handler boundaries are missing.'
}
$motionFrameBody = [string]::Join("`n", $dualBoardLines[($motionFrameStartLine - 1)..($controlFrameStartLine - 2)])
if ($motionFrameBody -notmatch 'last_rx_time_ms_\s*=\s*static_cast<uint32_t>\(\s*LibXR::Timebase::GetMilliseconds\(\)\s*\)') {
  throw 'A received MotionFrame must refresh the existing DualBoard link timestamp.'
}
if ($motionFrameBody -notmatch 'online_\s*=\s*true') {
  throw 'A received MotionFrame must establish the existing DualBoard online state.'
}
if ($motionFrameBody -notmatch 'safe_state_published_\s*=\s*false') {
  throw 'A received MotionFrame must re-arm the existing DualBoard offline safe state.'
}
if ($motionFrameBody -notmatch 'motion_state_\.yaw_rate_valid') {
  throw 'MotionFrame validity must be represented in ChassisMotionState.'
}
if ($motionFrameBody -notmatch 'motion_state_\.online\s*=\s*true') {
  throw 'MotionFrame must publish online state.'
}
if ($motionFrameBody -notmatch 'PublishMotionStateLocked\(\)') {
  throw 'MotionFrame must publish the merged semantic state.'
}
Assert-Contains $dualBoardHeader 'motion_state_ = {}' 'DualBoard timeout must clear the complete motion state.'
if ($dualBoardLines -notmatch 'event_id == static_cast<uint32_t>\(ChassisMode::ROTOR\)') {
  throw 'DualBoard must translate rotor mode semantically.'
}
if ($dualBoardLines -notmatch 'ChassisMotionMode::NON_ROTOR') {
  throw 'DualBoard must publish non-rotor mode semantically.'
}
Assert-Contains $dualBoardHeader 'PublishSafeChassisState' 'DualBoard must publish a zero command when the link is offline.'
Assert-Contains $dualBoardHeader 'ForceRemoteMode\(static_cast<uint32_t>\(ChassisMode::RELAX\)\)' 'DualBoard must force chassis RELAX on link timeout.'
Assert-Contains $dualBoardHeader 'SendClassicFrame' 'DualBoard must centralize CAN send error handling.'
Assert-Contains $dualBoardHeader 'can_->AddMessage\(pack\) == LibXR::ErrorCode::OK' 'DualBoard must check CAN AddMessage return values.'
Assert-NotContains $dualBoardHeader 'Topic::Server' 'DualBoard must not parse high-frequency motion data through packed Topic packets.'
Assert-NotContains $dualBoardHeader 'PackRaw' 'DualBoard must not forward high-frequency motion data as packed Topic packets.'
Assert-NotContains $dualBoardHeader 'FRAME_MAGIC' 'DualBoard must not keep the old fragmented Topic frame protocol.'
Assert-Contains $dualBoardHeader 'GetEvent' 'DualBoard must expose GetEvent for EventBinder.'
Assert-Contains $dualBoardHeader 'dualboard_chassis_mode' 'DualBoard mode topic default is missing.'
Assert-Contains $dualBoardHeader '0x312' 'DualBoard default gimbal TX ID is missing.'
Assert-Contains $dualBoardHeader '0x311' 'DualBoard default chassis TX ID is missing.'
Assert-Contains $dualBoardHeader 'chassis_cmd' 'DualBoard chassis command topic bridge is missing.'
Assert-Contains $dualBoardHeader 'yawmotor_angle' 'DualBoard yaw angle topic bridge is missing.'
Assert-Contains $dualBoardHeader 'pitchmotor_angle' 'DualBoard pitch angle topic bridge is missing.'
Assert-Contains $dualBoardHeader 'gimbal_euler' 'DualBoard attitude topic bridge is missing.'
Assert-Contains $dualBoardHeader 'launcher_ref' 'DualBoard launcher referee bridge is missing.'
Assert-Contains $dualBoardHeader 'sentry_ref' 'DualBoard sentry referee topic compatibility is missing.'

Assert-Contains $dualBoardCMake 'target_include_directories\(xr PUBLIC \$\{CMAKE_CURRENT_LIST_DIR\}\)' 'DualBoard CMake include path registration is missing.'
Assert-NotContains $dualBoardCMake 'XR_MODULE_DEPS' 'DualBoard CMake should not keep placeholder XR_MODULE_DEPS target.'

Assert-Contains $gimbalYaml 'name: DualBoard' 'Gimbal YAML must instantiate DualBoard.'
Assert-Contains $gimbalYaml 'ROLE: DualBoardRole::GIMBAL' 'Gimbal YAML must use GIMBAL role.'
Assert-Contains $gimbalYaml 'tx_id: 0x312' 'Gimbal YAML must use default gimbal TX ID.'
Assert-Contains $gimbalYaml 'rx_id: 0x311' 'Gimbal YAML must use default gimbal RX ID.'
Assert-Contains $gimbalYaml 'target_module: dual_board' 'Gimbal EventBinder must target DualBoard for chassis events.'
Assert-NotContains $gimbalYaml 'name: Chassis' 'Gimbal YAML must not instantiate Chassis.'
Assert-NotContains $gimbalYaml 'name: Referee' 'Gimbal YAML must not instantiate Referee.'
Assert-NotContains $gimbalYaml 'name: PowerControl' 'Gimbal YAML must not instantiate PowerControl.'
Assert-NotContains $gimbalYaml 'name: SuperPower' 'Gimbal YAML must not instantiate SuperPower.'

Assert-Contains $chassisYaml 'name: DualBoard' 'Chassis YAML must instantiate DualBoard.'
Assert-Contains $chassisYaml 'ROLE: DualBoardRole::CHASSIS' 'Chassis YAML must use CHASSIS role.'
Assert-Contains $chassisYaml 'tx_id: 0x311' 'Chassis YAML must use default chassis TX ID.'
Assert-Contains $chassisYaml 'rx_id: 0x312' 'Chassis YAML must use default chassis RX ID.'
Assert-Contains $chassisYaml 'chassis: ''@&chassis''' 'Chassis YAML must pass the chassis module to DualBoard.'
Assert-Contains $chassisYaml 'name: Chassis' 'Chassis YAML must instantiate Chassis.'
Assert-Contains $chassisYaml 'name: Referee' 'Chassis YAML must instantiate Referee.'
Assert-Contains $chassisYaml 'name: BMI088' 'Chassis YAML must instantiate BMI088.'
Assert-Contains $chassisYaml 'gyro_topic_name: chassis_gyro' 'Chassis BMI088 gyro Topic is missing.'
Assert-Contains $chassisYaml 'accl_topic_name: chassis_accl' 'Chassis BMI088 accel Topic is missing.'
$bmi088BlockLine = (Select-String -Path $chassisYaml -Pattern '^- id: BMI088_0$' -CaseSensitive).LineNumber
$dualBoardBlockLine = (Select-String -Path $chassisYaml -Pattern '^- id: dual_board$' -CaseSensitive).LineNumber
if ($null -eq $bmi088BlockLine -or $null -eq $dualBoardBlockLine -or $bmi088BlockLine -ge $dualBoardBlockLine) {
  throw 'Chassis BMI088 must be constructed before DualBoard subscribes to chassis_gyro.'
}
Assert-NotContains $chassisYaml 'name: DR16' 'Chassis YAML must not instantiate DR16.'
Assert-NotContains $chassisYaml 'name: Gimbal' 'Chassis YAML must not instantiate Gimbal.'
Assert-NotContains $chassisYaml 'name: InfantryLauncher' 'Chassis YAML must not instantiate InfantryLauncher.'
Assert-NotContains $chassisYaml 'name: SharedTopic' 'Chassis YAML must not instantiate SharedTopic.'

Write-Output 'PASS: DualBoard static regression checks'
