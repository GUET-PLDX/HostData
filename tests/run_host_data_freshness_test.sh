#!/usr/bin/env bash

set -euo pipefail

cxx=${CXX:-c++}
binary=$(mktemp /tmp/host_data_freshness_test.XXXXXX)

"$cxx" -std=c++17 -Wall -Wextra -Werror \
  tests/host_data_freshness_test.cpp -o "$binary"
"$binary"

printf 'PASS: HostData freshness helper regression\n'
