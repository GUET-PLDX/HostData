#!/usr/bin/env bash
set -euo pipefail

platform=""
case "$(uname -s)" in
  Linux)
    platform="linux"
    ;;
  Darwin)
    platform="darwin"
    ;;
  *)
    echo "Error: STM32 Cube bundle installation is supported on Linux and macOS only." >&2
    exit 1
    ;;
esac

case "$(uname -m)" in
  x86_64|amd64)
    arch="x86_64"
    ;;
  arm64|aarch64)
    arch="arm64"
    ;;
  *)
    echo "Error: unsupported host architecture: $(uname -m)." >&2
    exit 1
    ;;
esac

extension_root="${VSCODE_EXTENSIONS:-${HOME}/.vscode/extensions}"
cube_bin=""
for extension_dir in "${extension_root}"/stmicroelectronics.stm32cube-ide-core-*; do
  candidate="${extension_dir}/resources/binaries/${platform}/${arch}/cube"
  if [[ -x "${candidate}" ]]; then
    cube_bin="${candidate}"
    break
  fi
done

if [[ -z "${cube_bin}" ]]; then
  echo "Error: STM32 VS Code extension CLI was not found under ${extension_root}." >&2
  echo "Install the STM32 VS Code extension or set VSCODE_EXTENSIONS to its extension directory." >&2
  exit 1
fi

exec "${cube_bin}" bundle --yes install cmake ninja gnu-tools-for-stm32 st-arm-clang
