#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TOOL_ROOT="${XROBOT_TOOL_ROOT:-${REPO_ROOT}/.tooling/python}"

if ! command -v python3 >/dev/null 2>&1; then
  echo "Error: Python 3 is required to install xrobot tooling." >&2
  exit 1
fi

mkdir -p "${TOOL_ROOT}"
python3 -m pip install \
  --disable-pip-version-check \
  --target "${TOOL_ROOT}" \
  --upgrade \
  libxr xrobot

cat <<EOF
Python build tooling installed in: ${TOOL_ROOT}

Before running xrobot commands in this shell, export:
  export PATH="${TOOL_ROOT}/bin:\$PATH"
  export PYTHONPATH="${TOOL_ROOT}:\${PYTHONPATH:-}"

The STM32 compiler still requires either STM32 VS Code extensions (auto-detected
by tools/build.sh) or explicit GCC_TOOLCHAIN_ROOT and CLANG_GCC_CMSIS_COMPILER.
EOF
