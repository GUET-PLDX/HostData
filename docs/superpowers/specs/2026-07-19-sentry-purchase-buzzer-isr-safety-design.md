# Sentry Purchase and Buzzer ISR Safety Design

## Goal

Correct the sentry purchase command encoding without changing the application-level
meaning of its topics, and ensure the buzzer fatal-error callback never performs a
blocking operation in ISR context.

## Scope

This design changes only the sentry purchase path in `SentryProtocol`/`Referee`, the
fatal callback in `BuzzerAlarm`, and focused root-level regression tests. It does not
change BMI088, CameraSync, LibXR, YAML module composition, or unrelated referee
protocol fields.

## Sentry Purchase Semantics

`sentry_buy_bullet_num` carries one local purchase increment as `uint16_t`. It is not
the cumulative wire value. `Referee` owns the conversion from an increment to the
monotonically increasing 11-bit `buy_bullet_num` field in command `0x0120`.

The existing setter will be replaced by:

```cpp
LibXR::ErrorCode AddNeedBullet(uint16_t bullet_delta);
```

The operation holds `tx_data_mutex_` while reading and updating the cumulative field.
It applies these rules atomically:

- `bullet_delta == 0` returns `LibXR::ErrorCode::ARG_ERR`.
- `current + bullet_delta > 0x7FF` returns
  `LibXR::ErrorCode::OUT_OF_RANGE` without modifying state.
- Otherwise it stores the checked sum and returns `LibXR::ErrorCode::OK`.

`SentryProtocol::OnBuyBulletTopic` keeps reading a `uint16_t`, passes it without a
narrowing cast, and sends `0x0120` only after `AddNeedBullet` returns `OK`. Rejected
requests do not send an unchanged packet.

## Remote Purchase Semantics

The remote purchase topic is a trigger. A nonzero `uint8_t` requests exactly one
increment of the protocol's 4-bit remote-purchase request counter; the payload is not
an amount of ammunition.

The existing setter will be replaced by:

```cpp
LibXR::ErrorCode RequestRemoteBulletExchange();
```

The operation holds `tx_data_mutex_`, increments `remote_buy_bullet_times` by exactly
one, and never modifies `buy_bullet_num`. A counter already equal to `0x0F` returns
`OUT_OF_RANGE` without changing state. `OnRemoteBuyBulletTopic` ignores a zero trigger
and sends only after a successful request.

This keeps the two `0x0120` protocol fields independent and preserves their required
monotonic behavior.

## Buzzer Fatal Callback

The registered fatal callback checks `in_isr` before any PWM or sleep operation:

```cpp
if (in_isr) {
  return;
}
alarm->Play(alarm->alarm_freq_, alarm->alarm_duration_);
LibXR::Thread::Sleep(alarm->alarm_delay_);
```

The ISR path performs no buzzer output, delay, allocation, logging, or deferred work.
This is deliberate: the current LibXR fatal ISR path does not guarantee that a worker
thread can run. The non-ISR alarm sequence remains unchanged.

No changes are made under `Middlewares/`.

## Error Handling

Invalid or unrepresentable purchase requests are rejected rather than truncated,
wrapped, or saturated. The caller uses the returned `ErrorCode` to decide whether to
send the command. This prevents a packet from claiming a different purchase than the
application requested.

No logging is added inside Topic callbacks because their callback context is not
guaranteed to be safe for blocking output.

## Verification

Add focused root-level static regression tests consistent with the repository's
existing test style:

- Verify the local purchase path remains `uint16_t` end to end and contains no
  narrowing cast.
- Verify checked handling at zero, 255, 256, 1000, 2047, and values exceeding the
  remaining 11-bit capacity.
- Verify remote requests do not update `buy_bullet_num` and cannot wrap the 4-bit
  counter.
- Verify rejected setters do not call `SendSentryPack`.
- Verify the Buzzer ISR branch returns before `Play` and every `Thread::Sleep` call.

Run the focused tests, the repository format check, and compile both
`sentry_gimbal.yaml` and `sentry_chassis.yaml` with formatting skipped. Formatting is
applied only to changed files under `Modules/` using the required clang-format
version.

## Compatibility

The Topic names and payload widths remain unchanged. The C++ setter names change to
make increment and trigger semantics explicit; all in-repository call sites are
updated together. No generated files or vendor sources are edited.
