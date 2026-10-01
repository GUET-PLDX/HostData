#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
module_dir=$(cd -- "$script_dir/.." && pwd)
cxx=${CXX:-c++}
test_dir=$(mktemp -d /tmp/host_data_freshness_test.XXXXXX)
binary="$test_dir/host_data_freshness_test"
trap 'rm -rf "$test_dir"' EXIT

# Test the production state machine without the embedded runtime dependencies.
{
  printf '#include <cmath>\n#include <cstdint>\n#include <utility>\n'
  printf '#include "NavHostData.hpp"\n'
  sed -n '/^namespace Pldx::HostDataDetail {/,/^}  \/\/ namespace Pldx::HostDataDetail/p' \
    "$module_dir/HostData.hpp"
} > "$test_dir/host_data_detail.inc"

"$cxx" -std=c++20 -Wall -Wextra -Werror \
  -I"$test_dir" \
  -I"$module_dir/../CMD" \
  -I"$module_dir/../NavHostData" -I"$module_dir/../../Middlewares/Third_Party/LibXR/src/core" \
  "$script_dir/host_data_freshness_test.cpp" -o "$binary"
"$binary"
bash "$script_dir/host_data_single_owner_static_regression.sh"

printf 'PASS: HostData freshness and production wiring regression\n'
