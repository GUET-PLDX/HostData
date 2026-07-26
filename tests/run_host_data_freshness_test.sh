#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
module_dir=$(cd -- "$script_dir/.." && pwd)
cxx=${CXX:-c++}
binary=$(mktemp /tmp/host_data_freshness_test.XXXXXX)
trap 'rm -f "$binary"' EXIT

"$cxx" -std=c++20 -Wall -Wextra -Werror \
  -I"$module_dir/../NavLinkProtocol" \
  "$script_dir/host_data_freshness_test.cpp" -o "$binary"
"$binary"
"$script_dir/host_data_single_owner_static_regression.sh"

printf 'PASS: HostData freshness and production wiring regression\n'
