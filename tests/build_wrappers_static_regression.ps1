$ErrorActionPreference = 'Stop'

function Assert-Contains {
  param(
    [string]$Path,
    [string]$Pattern,
    [string]$Message
  )

  if (-not (Select-String -Path $Path -Pattern $Pattern -Quiet -CaseSensitive -SimpleMatch)) {
    throw $Message
  }
}

function Assert-ExecutableScript {
  param(
    [string]$Path,
    [string]$ConfigPath,
    [string]$DefaultBuildDir
  )

  if (-not (Test-Path $Path)) {
    throw "Missing build wrapper: $Path"
  }

  Assert-Contains $Path '#!/usr/bin/env bash' "$Path must be a bash script."
  Assert-Contains $Path 'tools/build.sh' "$Path must delegate to tools/build.sh."
  Assert-Contains $Path $ConfigPath "$Path must select $ConfigPath."
  Assert-Contains $Path $DefaultBuildDir "$Path must set default build directory $DefaultBuildDir."
  Assert-Contains $Path '"$@"' "$Path must forward user arguments."
}

Assert-ExecutableScript 'tools/buildgimbal.sh' 'User/RobotConfig/sentry_gimbal.yaml' 'build/sentry_gimbal'
Assert-ExecutableScript 'tools/buildchassis.sh' 'User/RobotConfig/sentry_chassis.yaml' 'build/sentry_chassis'

Write-Output 'PASS: build wrapper static checks'

