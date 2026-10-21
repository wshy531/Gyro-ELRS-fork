# Independent flapping mode / 独立扑翼模式

This mode drives two independently actuated wings for manual forward-flight experiments. It does not provide attitude stabilization or prove that a particular airframe can fly. Enable it from the matching configurator; no phone-app changes are required.

## Configuration

`GET /config` exposes `config.fc_flapping` on flight-control receivers with PWM output support. `POST /config` accepts the same object. Settings are stored as `flapping` in the existing `/fc.json` file; missing or invalid settings default to disabled.

- `enabled`: defaults to false. ARM must be configured; drift assistance cannot be enabled at the same time.
- `throttle_channel`, `steering_channel`: distinct 1-based RC channels, default CH3 / CH1. Neither may conflict with ARM or enabled Wi-Fi coexistence.
- `min_hz`, `max_hz`: defaults 0.5 / 2 Hz, software range 0.1–10 Hz. The fastest half-stroke must contain at least four PWM refresh periods on both outputs. This is a sampling constraint, not a servo speed rating.
- `downstroke_percent`: default 35%, range 20–50%. `differential_percent`: default 20%, range 0–50%. `reverse_steering` defaults to false.
- `left`, `right`: each contains `output` (1-based physical PWM port), `up_us`, `down_us`, `park_us` (final pulse widths, 500–2500 μs and inside the port limits). Default ports are 1 / 2 and all positions are 1500 μs. Calibrate distinct upper/lower positions before enabling. Use standard PWM, not narrow pulses, DShot, digital or serial outputs. Reboot after changing PWM mode or refresh rate; the runtime guard uses the frequency actually allocated at startup.

## Behavior

Each half-stroke interpolates its endpoints with `(1-cos(pi*x))/2`. Both wings share a phase. Throttle at or below 1% stops; above that, its remaining range maps linearly onto frequency. Frequency changes do not restart the phase. Positive steering reduces the right downstroke excursion, negative steering reduces the left; reversal swaps this relationship. Downstroke targets are latched at the start of each cycle, so changes cannot move an endpoint in mid-stroke.

The existing ServoOutput scheduler advances the oscillator every 4 ms independently of radio packets and IMU/PID readiness. Wing position commands bypass ordinary source/reversal mapping, while signal polarity and physical output limits still apply. Other passthrough channels retain their original behavior. Other mixer outputs use their existing safe zero-input values while flapping is enabled.

Wing commands are submitted at most once per allocated PWM refresh period (20 ms at 50 Hz), while phase calculation remains at 4 ms. This bounds hardware update cadence; it has been checked with a host simulation of the ServoOutput path, including parking, manual-test priority and releasing the outputs. On 2026-10-21 the user confirmed motion after enabling signal polarity inversion on their hardware; the no-motion fault was a polarity mismatch, not a verified scheduling fault. Polarity depends on the receiver output circuit.

Startup requires valid RC/model match, ARM and observed zero throttle while armed. Starting moves from the current position to the upper endpoint over 300 ms. Zero throttle, disarming, RC loss, a timing gap above 100 ms or disabling the mode stops the cycle and interpolates to the old pair's parked positions over 300 ms. Parking continues without new RC packets and takes precedence over ordinary failsafe modes on wing outputs. Remapping acquires the new pair after parking the old pair.

RF freshness follows the existing LQ/absolute 1000 ms timeout. Pure Wi-Fi suspends motion and parks the wings. Existing PWM output testing takes priority; after testing, the last commanded positions are used for parking and zero throttle is required again. Configuration changes reset the startup gate.

`GET /status.json` includes `flapping`: the current startup blocker or motion state, received throttle/ARM pulse widths and channel numbers, commanded wing positions and frequency. These diagnostics do not change actuation rules. Received inputs may be stale after RC loss; position commands are not servo feedback. The zero-throttle gate currently requires at most 998 μs, so a transmitter minimum of 1000 μs will not pass it.

While connected, save changes only when disarmed, or while armed with old and new throttle inputs at or below 1% and both wings parked. This keeps RF/Wi-Fi coexistence available. Changes to an owned port's PWM mode or limits also wait for parking before they can be saved, including when disarmed or disconnected. Invalid settings are rejected before configuration setters run. Disconnected configuration is allowed.

## Checks

From `src`, run `pio test -e native -f test_flapping -f test_drift` and build `Unified_ESP32C3_2400_RX_via_WIFI`. Use separate `PLATFORMIO_BUILD_DIR` values if Native tests and firmware builds run concurrently. Noninteractive unified builds produce a bare image; select the correct hardware layout before flashing.

Tests cover asymmetric timing, differential/reversal, cycle latching, no cumulative amplitude reduction, throttle phase continuity, startup gating, parking, remapping, clock wrap, invalid input, JSON validation, PWM constraints and saving while armed at zero throttle. Actual servo tracking, loaded wing motion, radio/Wi-Fi coexistence and flight behavior still require hardware validation.

2026-10-20 software verification: 66 selected receiver Native cases passed (10 flapping + 56 existing regression cases); 11 configurator unit tests passed; mocked-browser beginner/expert saving, profile application, offline/legacy behavior and phone layout passed without browser errors. ESP32-C3 unified firmware and the Windows NSIS test installer built successfully. Known upstream failing/empty Native suites were excluded, so this is not a full Native suite pass. No hardware was flashed and no servo, radio coexistence or flight test was performed.

2026-10-21 final check: 67 selected Native cases passed, including the new runtime-state test; ESP32-C3 firmware built successfully. A host simulation of ServoOutput checked wing frame pacing, lost-link parking, reconnect gating, manual PWM test priority, disabled-mode RC passthrough and configured disarmed failsafe handling on other reversed mixer motor outputs. The host simulation substitutes PWM hardware and does not verify physical waveforms. The user reported working servos with the correct output polarity; loaded wings, failsafe on hardware and flight remain unverified.
