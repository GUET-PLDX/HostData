#!/usr/bin/env bash
set -euo pipefail

dr16_header="Modules/DR16/DR16.hpp"

thread_body="$(sed -n '/static void ThreadDr16/,/^  }/p' "$dr16_header")"
parse_body="$(sed -n '/LibXR::ErrorCode ParseRC/,/^  }/p' "$dr16_header")"
checkout_body="$(sed -n '/void CheckoutOffline/,/^  }/p' "$dr16_header")"

if ! grep -q 'cmd_->FeedRC(CMD::RCInputSource::RC_INPUT_DR16, rc_data)' \
  <<<"$thread_body"; then
  echo 'FAIL: ThreadDr16 must submit each successfully parsed frame to CMD.' >&2
  exit 1
fi

if grep -q 'cmd_->FeedRC' <<<"$parse_body"; then
  echo 'FAIL: ParseRC must not submit data to CMD.' >&2
  exit 1
fi

if ! grep -q 'if (!this->offline_latched_)' <<<"$checkout_body"; then
  echo 'FAIL: CheckoutOffline must notify CMD only once per offline period.' >&2
  exit 1
fi

echo 'PASS: DR16 input lifecycle static checks'
