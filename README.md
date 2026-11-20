# ODrive M0/M1 motor tests

This workspace targets an ODrive v3.6 clone with a firmware 0.5.1-compatible
interface. The scripts select one axis at a time and require both axes to be
IDLE before motion. Motion commands in this workspace share a lock. Use a single
controller for the board while testing.

## Wiring and first checks

- Power down before connecting motor phases, encoder, or brake resistor.
  Connect each motor and its encoder to the matching axis. Verify the encoder
  supply, signal levels, pinout, and common ground for the board and encoder.
  Use incremental A/B and leave Z disconnected for this workflow.
- M0 uses a 24 V supply, a 140 KV motor with seven pole pairs, and a 500 PPR
  quadrature encoder. Set `cpr` to **2,000 counts per mechanical turn** for
  that encoder. With the drive disabled, a one-turn measurement in the
  encoder monitor can confirm CPR and that A/B counts change smoothly.
- M1 starts with `hardware_verified: false`. Verify its motor KV and pole pairs,
  current and voltage ratings, encoder presence and CPR, and permitted speed.
  Fill in its own hardware, velocity, safety, and calibration values. Set
  `encoder_present: true` only for a connected A/B encoder. Then set
  `hardware_verified: true`. For an encoderless M1, leave `cpr`, `bandwidth`,
  `calib_range`, and `enable_phase_interpolation` null.
- Verify the shared supply and brake resistor against `board`. Mount the
  motor securely, unload and guard the shaft, and keep the power cutoff accessible.

## Commands

From `~/odrive_test`, use the project's existing virtual environment.

### Desktop test console

Search for **ODrive Bench** in the Ubuntu application menu, or run
`.venv/bin/python -m ui` from this workspace. For a fresh desktop
setup at this workspace path, install the launcher once with
`install -Dm644 ui/odrive-bench.desktop "${XDG_DATA_HOME:-$HOME/.local/share}/applications/odrive-bench.desktop"`.
The GUI uses Tkinter and Matplotlib in the workspace Python environment.
The desktop app and its assets live in `ui/`. Motor control and terminal
commands live in `motor/`. The config, logs, and tests stay at the
workspace root.

Use **Check drive** to refresh both axes, select an axis, and clear any
faults. **Calibrate A/B** runs calibration in RAM and requires an existing
incremental encoder configuration. Use the CLI for encoder-mode changes
and AS5048A setup. A previously calibrated AS5048A can still be used in
GUI speed tests when it passes preflight.

**Control** is the default. It runs `control_m0.py` on M0 and chooses
position below 200 RPM in magnitude or torque at 200 RPM and above.
**Velocity** runs `vel_m0.py` with encoder feedback. **Torque** runs
`torque_m0.py` on M0. **Encoder** opens the read-only `encoder_m0.py`
count monitor while IDLE, using **Monitor encoder** and **Stop monitor**.

Enter a signed RPM for a motor test and click **Run test** in the top-right corner. Review the
preflight details before clicking **Confirm motion**. **Stop test** or
**Esc** requests IDLE and the shaft coasts. If IDLE is unconfirmed,
isolate motor power and use **Check drive** to verify the state before
another run. Keep the physical power cutoff accessible during testing.

To change targets during a run, enable **Allow live RPM** before preparing
it. The live ceiling defaults to the magnitude of the starting target.
Review the ceiling, ramp, and firmware trip before confirming. Enter a
new signed target and click **Apply RPM**. Zero holds in closed loop.
Reversals ramp through zero and wait for both speed readings to settle.
Control keeps its starting mode and accepts targets on that side of the
200 RPM magnitude boundary, plus zero. A lost GUI heartbeat stops the
run through the same cleanup path as **Stop test**.

The **Teleop** panel under **Run events** uses the axis and mode selected
in Drive operations. Set an RPM ceiling, click **Activate Teleop**, and
review **Confirm motion**. Press **↑** or **↓** to drive, and hold an arrow
to increase speed within the ceiling. Releasing it keeps the last target.
The indicators show the controller's active direction, and reversals wait
for near-zero speed. Tap **Space** twice to brake and hold zero. **Stop
test**, **Esc**, switching windows, or minimizing ends Teleop and requests
IDLE. Control Teleop on M0 selects Position for ceilings below 200 RPM
and Torque for ceilings from 200 RPM up to the configured limit
(2,200 RPM by default). That mode stays fixed until the session ends.

**Tune parameters** opens a dock beside Drive operations with the gains and
ramps for the selected mode.
**Use for next run** stages a draft for one run, and **Save parameters**
updates the selected JSON defaults while the workspace motion lock is
held. If the JSON changed since the editor loaded it, use **Reload
parameters** and reapply the edits. Gains and ramps take effect at the
next confirmed start and the effective settings are recorded in the CSV.
**Advanced** shows the supported axis and shared config fields. Review the
old and new values before saving advanced edits. Board identity,
verification flags, and required protection switches are read-only.
Run **Check drive** again after an advanced save.

Speed, Current, and DC bus are the default graphs. Use the checkboxes
above each graph to toggle the graph and its individual traces. Uncheck
the graph name to collapse it, or check its name again to restore it.
Under **Optional graphs**, enable Torque, Speed error, Shaft rotation,
Position, Following error, Bus current, or Loop timing. These plots start
hidden and can be enabled when the CSV contains the corresponding signals.
Drag horizontally on a plot or move the timeline scrollbar to browse the
full recording in 30-second windows. **Follow live** returns to the newest
samples and **Freeze view** holds the current display. Recording continues
as you browse. Use the wheel or vertical scrollbar to scroll the telemetry
pane. Drag up or down inside a graph to pan just that graph. Ctrl+scroll
zooms the graph and double-click resets its view. **Open log folder** takes
you to the full-rate CSV files.

The app opens in a maximized window in Dark mode. Press **F11** to toggle fullscreen,
drag the dividers to resize the panes, or use **Dark mode** in the header
to change the theme.

### Terminal commands

| Task | M0 | M1 |
| --- | --- | --- |
| Read firmware capabilities | `.venv/bin/python -m motor.calibrate_m0 --check-config` | `.venv/bin/python -m motor.calibrate_m0 --axis 1 --check-config` |
| Clear faults and calibrate in RAM | `.venv/bin/python -m motor.calibrate_m0` | `.venv/bin/python -m motor.calibrate_m0 --axis 1` |
| Clear latched faults only | `.venv/bin/python -m motor.calibrate_m0 --clear` | `.venv/bin/python -m motor.calibrate_m0 --axis 1 --clear` |
| Velocity console | `.venv/bin/python -m motor.vel_m0` | `.venv/bin/python -m motor.vel_m0 --axis 1` |
| Terminal keyboard control | `.venv/bin/python -m motor.teleop_motor` | `.venv/bin/python -m motor.teleop_motor --axis 1` |
| Single signed encoder-velocity trial | `.venv/bin/python -m motor.vel_m0 -30` | `.venv/bin/python -m motor.vel_m0 --axis 1 -30` |
| Torque-mode speed trial | `.venv/bin/python -m motor.torque_m0 -30` | |
| Auto-select position or torque mode | `.venv/bin/python -m motor.control_m0 30` | |
| Monitor raw encoder counts while IDLE | `.venv/bin/python -m motor.encoder_m0` | `.venv/bin/python -m motor.encoder_m0 --axis 1` |
| Five-second sensorless diagnostic | `.venv/bin/python -m motor.vel_m0 --sensorless 550` | `.venv/bin/python -m motor.vel_m0 --axis 1 --sensorless 550` |

`--axis 0` is equivalent to omitting `--axis`. Example targets must be
within the selected axis's limits. Calibration measures phase resistance
and inductance, then aligns the A/B encoder and waits for the shaft to settle
when an encoder is configured. An encoderless axis performs motor calibration
only. Ordinary A/B calibration stores its results in RAM. Repeat it after
each boot.

The default command clears faults on the selected axis, then asks for
confirmation before calibration. Both axes must be IDLE. Use `--clear` to
clear faults while keeping the current calibration. A fault that returns
after clearing blocks calibration.

### M0 AS5048A absolute SPI encoder

With the AS5048A wired to SPI, run the following once with the GPIO connected
to its CSn pin. This example uses GPIO4.

```bash
.venv/bin/python -m motor.calibrate_m0_x --cs-gpio 4
```

The script saves the SPI setup and reboots, measures motor resistance and
inductance, and aligns the encoder. It saves both calibrations and verifies
them after another reboot with M0 in IDLE. The shaft may turn during calibration.
ODrive saves the whole board configuration to flash.

Use `.venv/bin/python -m motor.calibrate_m0_x --check-config` to see the current mode and readiness or
`.venv/bin/python -m motor.calibrate_m0_x --clear` to clear M0 faults. Velocity, teleop, position,
and torque commands use the active encoder. The AS5048A is absolute within
one revolution.

To return to A/B feedback, reconnect the quadrature encoder and run
`.venv/bin/python -m motor.calibrate_m0`. It saves the mode change, reboots, and then calibrates
in RAM. Repeat A/B calibration after each boot. Use A/B mode for the
sensorless bench test.

In the velocity console, enter a signed RPM and press Enter again to confirm
motion. Ctrl+C stops the trial and commands IDLE. The shaft coasts. Enter
`q` at the prompt to exit. Encoder velocity requires valid motor and encoder
offset calibration on the selected axis.

The sensorless test repeats motor R/L calibration before handoff at 550 RPM.
Use a signed target from 550 to 800 RPM within the axis's configured command
range. For M1, verify that the motor and load permit the 550 RPM handoff and
the requested target, fill in its sensorless PI gains, then set
`sensorless.verified: true`. M0 retains the original gains of 0.01 and 0.05.
The sensorless test can compare estimated turns with A/B counts when an encoder
is configured. Phase comparison also needs a prior A/B offset calibration.
Recalibrate the A/B offset after a sensorless test before using encoder velocity.
Add `--continuous` for the existing supervised run until Ctrl+C or a fault.

### M0 automatic position and torque control

Run `.venv/bin/python -m motor.control_m0 30` or omit the RPM for the repeat-test
console. The controller chooses position mode below 200 RPM in either
direction, and torque mode at 200 RPM and above. The mode is fixed for each trial.
Both modes rely on the ODrive firmware overspeed trip.
Position mode limits the target lead to 0.05 turns and the trial torque to
0.08 N·m, or the existing limit if lower. Ctrl+C requests IDLE.

Tune `axes.m0.controller.pos_gain` in `motor_config.json` in s⁻¹. The script
reads it before each trial, shows it at confirmation, and records it in the
CSV. Position trials use 20 when the setting is missing or `null`.
Tune the position-mode velocity P gain with
`axes.m0.velocity.position_vel_gain` (default 0.12 N·m per turn/s).

Position trials write `logs/m0_position_*.csv` with position, following error,
and velocity feed-forward. In these files, `setpoint_rpm` is the requested
ramp and `velocity_feedforward_rpm` is the speed actually sent for that tick
after applying the lead limit. The `active_*` fields show the commands in
effect when feedback was read. The companion CSV uses the same control-tick
samples and is enabled by default. Use `--no-capture` to skip it.

### M0 gain tuning from CSV logs

In ODrive Bench, select M0 and open **PSO tuning** below **Tune parameters**.
Choose Position or Torque, then **Analyze logs** to see model checks and
missing signed speeds. **Capture reference** records a timed run at a
signed RPM using the displayed gains. **Validate finalist** repeats the
offline check and requires a fresh confirmation for every motion trial.
Stop test or Esc aborts the sequence. ODrive Bench checks both axes for
IDLE and waits for M0 to settle between trials. Higher torque stages need a
separate guarded-speed confirmation. Review the per-speed evidence before
using **Save validated gains**. Saving updates only that mode's gain pair
in the workspace JSON.

Run the offline tuner with the bench environment's NumPy installation.

```bash
.venv/bin/python -m pso.pso_m0 position
.venv/bin/python -m pso.pso_m0 torque
```

Position tuning searches `controller.pos_gain` and
`velocity.position_vel_gain`. Torque tuning searches `torque.kp` and
`torque.ki`. The tuner fits compatible, completed trials in `logs/` and
reports held-out model errors and signed speeds that still need evidence.
Use `--logs-dir DIR` to analyze archived CSVs in place. Torque optimization
can show provisional rankings at independently qualified low or moderate
signed targets. These rankings are for offline review. A torque finalist
for the bench requires full qualification at both signs of 200, 500, 1,000,
2,000, and 2,200 RPM. Obtain high-speed CSVs with separately supervised
fixed-target Control trials.

Analysis keeps repeated holdouts and reports current-limit fault masks.
A current-limit fault blocks the affected signed target and runtime setup
until two later, complete fault-free captures match the saved gains and
that setup.

Position replay checks raw count-based RPM at the original CSV timestamps,
the four tracking metrics, a 250 ms speed-trend RMSE, and the dominant
plateau oscillation above 1 Hz within the sampled band. Trend RMSE uses
the raw RMSE limit. Oscillation amplitude uses the larger of 1 RPM or 20%
of the measured peak amplitude. If both peaks reach 1 RPM, the frequency
limit is the larger of one FFT bin or 20% of the measured peak frequency.
A raw trace failure still blocks qualification. The report also shows
whether the measured-current rotor replay completed the same window, and
compares firmware torque predicted from the logged active commands and
feedback against the observed torque. Add `--diagnostics-dir DIR` to write
aligned holdout traces for plotting, even when qualification fails.
New position logs record the initial encoder state, an Iq read timestamp,
and host timestamps bracketing each of the position and feed-forward
writes. Replay uses the midpoint of each write bracket as an estimate.
Older CSVs use approximate timing and remain available for diagnostics.
Host sampling does not resolve firmware-rate changes or frequencies above Nyquist.

If ±30 RPM position holdouts fail, compare the normal closed-loop replay
with the rotor driven directly by the recorded `Iq_measured` torque estimate.

```bash
.venv/bin/python -m pso.pso_m0 position --measured-current-replay
```

The diagnostic fits the rotor on training runs and evaluates independent
holdouts. It checks the logged torque conversion and count-derived RPM,
then compares free-running replay with a one-step replay that starts each
interval from the recorded encoder count and firmware speed estimate.
It tests linear interpolation and a hold of the last current sample.
For legacy USB captures it repeats those comparisons at three estimated
times based on the USB read order. It also compares against
carrying forward the initial speed without torque. Early stops limit the
free-running comparison to shared samples. The report checks the fitted
rotor parameters by leaving out each training recording. It splits one-step
errors and changes in adjacent count-derived speeds into startup (first
second), acceleration ramp, settling, and the existing steady window.
It flags a phase as under-resolved when the same pair of current
reconstructions differs by more than 6 RPM RMS on at least half the
holdouts for that signed target and phase. This diagnostic does not qualify
the model or rank gains. Use `--logs-dir DIR` for archived captures.

#### Import a synchronized M0 recording

Position qualification uses native trials with recorded write timing and
matching synchronized recorder exports. Keep the native CSV intact. Put
`<sha256>.csv` and `<sha256>.json` in `logs/position_recorder/`, using the
lowercase SHA-256 of the native CSV as the basename. Both the CLI and ODrive
Bench use that folder by default. With `--logs-dir`, offline analysis looks
for `position_recorder/` inside that log directory.
The export CSV header is `tick,encoder_shadow_count,iq_setpoint_a,iq_measured_a`.
Use an unwrapped integer device tick and the raw signed 32-bit encoder count.
Both Iq values must be in amperes, with the drive's original polarity.

The JSON manifest contains `native_sha256`, `axis` (0), `board_serial`,
`encoder_cpr`, `overflow` (false), `tick_hz`, `tick_stride`, and two
`clock_anchors`. Each anchor contains `tick` and `elapsed_s` in the native
trial's time base. Use a verified marker before and after the recording,
and bracket all exported ticks. A stride of 40 at 8,000 ticks per second
gives 200 frames per second. The importer requires that rate or higher,
consecutive frames at the declared stride, and coverage from the native
trial's time zero through its recorded stop. It checks clock-rate agreement
within 1% and verifies at least 95% of native counts against the bracketing
recorder counts with a one-count allowance.

```bash
.venv/bin/python -m pso.pso_m0 position --recorder-dir /path/to/exports --measured-current-replay
.venv/bin/python -m pso.pso_m0 position --recorder-dir /path/to/exports
```

The first command diagnoses matched ±30 RPM holdouts. Without an explicit
`--recorder-dir`, that diagnostic uses native USB samples. The second fits the
model on matched training exports at two gain pairs. Holdout recorder
current is used only for the rotor replay. Closed-loop validation predicts
the response from the native holdout setup, gains, and timing. A clean
legacy capture without synchronized data can still be diagnosed separately.
The included `pso.usb_recorder` records only while both axes are IDLE.

Use `--reference RPM` for one supervised fixed-target capture. Add
`--gains FIRST SECOND` to test a pair in the order listed above. Stop with
Ctrl+C after the capture duration shown in the confirmation prompt. The
reference applies its gains in memory for that run.
Start qualification with two synchronized captures per gain pair and
direction at +30 and −30 RPM, using the saved pair and `--gains 8.0 0.20`
at the same admitted period and torque cap. Two passing holdouts per signed
setup are required. Position bench validation requires coverage at both
signs of 1, 10, 30, and 100 RPM.

Add `--bench` to validate an offline finalist. Confirm each trial and stop
with Ctrl+C after its measurement window. Torque validation stages ±2,000
RPM before ±2,200 RPM, with a separate guarded confirmation for each
direction. After the bench checks pass, type `save` to apply the pair.

### M0 torque-mode speed control

Calibrate M0 after powering on, then run `.venv/bin/python -m motor.torque_m0 -30`
for a single signed RPM trial, or omit the RPM for the repeat-test console.
The script times USB feedback with both axes IDLE. It tries 40 Hz, falls back
to 30 Hz if needed, and refuses to arm when both miss the timing budget.
Press Enter to confirm the rate and start the trial. It runs until Ctrl+C
or a fault, then requests IDLE and the shaft coasts.
Enter `q` at the RPM prompt to exit.

Tune `axes.m0.torque` in `motor_config.json`. The console rereads it before each
trial. `torque_limit_nm` is the requested drive and braking cap. `kp` is in
N·m per turn/s, and `ki` is in N·m per turn/s per second.
`slew_rate_nm_per_s` limits host torque changes in both directions, defaulting
to 1.0 N·m/s. A drop in the live torque cap takes effect immediately.
`firmware_limiter_gain` controls how sharply the ODrive changes its torque
limit with speed. Its starting value is 0.40 N·m per turn/s.

`overspeed_ratio` defaults to 1.20 and must be at least 1.0. The firmware trip
is this ratio times the greater of `controller_min_limit_rpm` and the target
magnitude plus `controller_headroom_rpm`. At a 1,000 RPM target with 20 RPM of
headroom, the trip is 1,224 RPM. Older configs inherit the shared
`velocity.vel_limit_tolerance`.

Python increases drive torque when speed drops under load. The confirmation
shows the trial cap, estimated drive torque available at the target, and
firmware trip. The firmware tapers available torque near the speed ceiling
and can also derate it as the drive heats up. The stall timer starts once
the firmware torque setpoint reaches the available drive torque.

Set `hardware.torque_constant` in N·m/A to override the estimate
`8.27 / motor_kv`. Leave it `null` to use the estimate. Run calibration again
after editing it. Set `controller.torque_ramp_rate` to a positive value in N·m/s
to use the ODrive's firmware input torque ramp. With `null`, Python uses
passthrough. The CSV records the live torque constant, input mode, and rate.

The trial CSV and default companion `logs/m0_capture_<same UTC time>_<signed target>rpm.csv`
share one snapshot per control tick. Use `--no-capture` to omit the companion
file. A separate writer handles the CSV and terminal output. Rows include
measured `Id` and `Iq`, host and inferred firmware torque, the PI terms,
`loop_interval_s`, `read_span_s`, bus voltage and `ibus_a`, and whether the
brake output is armed or saturated. The trial stop row records the stop
reason and labels the fault masks read after IDLE in `post_stop_faults_json`.
Its main fault columns use the in-loop fault if available, then M0's post-stop
reading. USB polling can miss brief transients between reads.

| Display | Meaning |
| --- | --- |
| Torque Nm | New Python request. |
| Prev Nm | Previous Python request. |
| FW Nm | Firmware torque setpoint inferred from `Iq_setpoint` and aligned with the previous request. |
| I held | The integral term paused at a host or firmware torque bound in the direction of the speed error. |

For a first guarded bench check, set `torque_limit_nm` to 0.10 and start at
30 RPM with the shaft guarded and the brake resistor working.
For overspeed diagnosis at 1.20, advance through 300, 500, and 1,000 RPM after
stable trials and compare logged speed with independent shaft-speed measurements.
Verify the motor and attached mechanism's maximum shaft speed before raising
the ratio. Before robot use, verify repeatable starts, steady holds under
representative load, and that a fault or command-link loss leaves the motor
disarmed until an explicit restart.

## Terminal keyboard control

Run teleop in an interactive terminal after calibrating the selected axis.
The drive starts at zero after confirmation. **Releasing an arrow keeps the
last commanded speed.**

```bash
.venv/bin/python -m motor.teleop_motor
.venv/bin/python -m motor.teleop_motor --mode control --max-rpm 30
.venv/bin/python -m motor.teleop_motor --mode torque --max-rpm 250
```

| Key | Action |
| --- | --- |
| Up / Down | Command forward / reverse. Hold to accelerate faster up to the configured ramp and approved speed ceiling. An opposite arrow requests reversal after the shaft settles near zero. |
| Space twice within 300 ms | Command zero immediately, bypassing the driving speed ramp, then hold at zero. |
| `Ctrl+C` | Request IDLE immediately and exit. The shaft coasts. |

Velocity is the default and supports `--axis 1` on a configured M1. On M0,
Control uses Position for a ceiling below 200 RPM and Torque for a ceiling
of 200 RPM or more. Explicit Torque is also available on M0. The default
ceiling is 50 RPM, bounded by the selected axis's configured command range.
The effective mode and ceiling stay fixed for the run.

The terminal uses keyboard repeat to detect a hold. Double Space timing
assumes the GNOME defaults of a 500 ms repeat delay and 30 ms repeat interval.
After braking, let the shaft settle and leave the arrow keys untouched for
750 ms before pressing an arrow to resume. A fault, disconnect, or three-second
braking timeout requests IDLE and ends the session. Physical stopping time
depends on the load and the drive limits. Braking feeds energy back into the
DC bus, so the brake resistor must be sized for the load.
[ODrive 0.5.1 braking guidance](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.1/docs/getting-started.md#hardware-requirements)

Telemetry goes to the selected runner's CSV in `logs/`, with a `teleop`,
`position`, or `torque` filename prefix after the axis number. The filename
RPM is the approved ceiling. Samples include the requested and active RPM
targets alongside the setpoint, measured speed, current and faults.

## Configuration

`motor_config.json` has a board serial, shared `board` DC-bus settings, shared
`runtime` timeouts, and `axes.m0` and `axes.m1`. Each axis has its own
`hardware`, `velocity`, `controller`, `anticogging`, `sensorless`, `safety`,
and `calibration` settings. Legacy JSON with top-level M0 settings still loads
for M0. In the new schema the original M0 values are under `axes.m0`; the
five shared DC-bus settings have moved from `hardware` to `board`.
`board.dc_max_positive_current: null` selects the firmware's unlimited default.
The existing brake resistor is 2 Ω and the configured overvoltage trip is 28 V.

### Firmware settings

`hardware.motor_kv` is RPM/V and yields the torque constant `8.27 / KV`
in Nm/A. `pole_pairs` and `cpr` are positive integers. `current_lim`,
`current_lim_margin`, and `calibration_current` are in amperes.
`resistance_calib_max_voltage` is in volts. `bandwidth` is the encoder PLL
bandwidth in 1/s. `calib_range` is the fractional tolerance for the encoder
calibration scan. M0 retains a 10 A current limit, 10 A calibration current,
140 1/s encoder bandwidth, and 0.02 calibration tolerance.

The table lists JSON fields and their destination under the selected axis.
For optional settings, `null` retains the live value. Stock defaults are shown
for comparison with the clone's live values. The three protection switches
must stay true. Keep anticogging off unless the selected axis already has
a valid map for its current alignment. The encoder-velocity trial checks
the firmware's `anticogging_valid` flag before enabling it and disables it
again at IDLE. Calibration and sensorless runs keep anticogging off.

| JSON group | Field | Firmware path | Units or effect | Shipped setting / stock 0.5.1 default |
| --- | --- | --- | --- | --- |
| hardware | `current_control_bandwidth` | `motor.config.current_control_bandwidth` | rad/s, inner current-loop PI gains | null / 1,000 |
| hardware | `torque_lim` | `motor.config.torque_lim` | Nm, torque ceiling along with current limiting | null / unlimited |
| hardware | `torque_constant` | `motor.config.torque_constant` | Nm/A, applied when configuring calibration | null / estimate from motor KV |
| hardware | `enable_phase_interpolation` | `encoder.config.enable_phase_interpolation` | Interpolate electrical phase between counts | null / true |
| controller | `pos_gain` | `controller.config.pos_gain` | (turn/s)/turn, position loop P gain | M0: 20, M1: null / 20 |
| controller | `inertia` | `controller.config.inertia` | Nm/(turn/s²), acceleration feed-forward in velocity ramp, position filter, and trajectory modes | null / 0 |
| controller | `input_filter_bandwidth` | `controller.config.input_filter_bandwidth` | 1/s, position input filter | null / 2 |
| controller | `enable_gain_scheduling` | `controller.config.enable_gain_scheduling` | Scale PI gains around zero position error in position control | null / false |
| controller | `gain_scheduling_width` | `controller.config.gain_scheduling_width` | turns, width of the position error region | null / 10 |
| controller | `enable_vel_limit` | `controller.config.enable_vel_limit` | Clamp commanded velocity | true / true |
| controller | `enable_overspeed_error` | `controller.config.enable_overspeed_error` | Trip on estimated velocity beyond `vel_limit × vel_limit_tolerance` | true / true |
| controller | `enable_current_mode_vel_limit` | `controller.config.enable_current_mode_vel_limit` | Velocity limiting in torque/current control | true / true |
| controller | `torque_ramp_rate` | `controller.config.torque_ramp_rate` | Nm/s in torque-ramp input mode | null / 0.01 |
| anticogging | `anticogging_enabled` | `controller.config.anticogging.anticogging_enabled` | Allow an existing valid map to supply torque feed-forward | false / true |
| anticogging | `calib_pos_threshold` | `controller.config.anticogging.calib_pos_threshold` | encoder counts of position error | null / 1 |
| anticogging | `calib_vel_threshold` | `controller.config.anticogging.calib_vel_threshold` | encoder counts/s of speed | null / 1 |

All listed paths are exported by the official 0.5.1 interface. The
`--check-config` command reads each field on the selected clone axis and reports
its live value or `unsupported`. A missing field with an explicit setting
blocks the trial, while a missing field configured as null is reported and
skipped. Availability was verified against upstream source offline. Use the
read-only command to verify what this board exposes.

[0.5.1 firmware interface](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.1/Firmware/odrive-interface.yaml) ·
[Controller source](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.1/Firmware/MotorControl/controller.hpp) ·
[Motor source](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.1/Firmware/MotorControl/motor.hpp) ·
[Encoder source](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.1/Firmware/MotorControl/encoder.hpp) ·
[Control modes](https://github.com/odriverobotics/ODrive/blob/fw-v0.5.1/docs/control.md)

### Velocity and Python diagnostics

| Fields | Units and behavior |
| --- | --- |
| `vel_gain`, `vel_integrator_gain` | Firmware velocity PI gains in Nm/(turn/s) and Nm/((turn/s)·s). M0 encoder defaults are 0.085 and 0.15. |
| `vel_ramp_rate`, `fast_vel_ramp_rate`, `fast_ramp_above_rpm` | Ramp rates in (turn/s)/s. Python selects the fast ramp above the configured RPM threshold. M0 uses 1 turn/s² for both, with a 100 RPM boundary. |
| `min_command_rpm`, `max_command_rpm` | Python command magnitude bounds. M0 accepts 1–2,200 RPM. |
| `controller_min_limit_rpm`, `controller_headroom_rpm`, `vel_limit_tolerance` | Python converts the greater of the minimum and target magnitude plus headroom to firmware turns/s, capped by `max_command_rpm`. The dimensionless tolerance sets the firmware overspeed trip. M0 uses 20 RPM, 20 RPM, and 1.2. |
| `software_min_limit_rpm`, `software_max_command_factor`, `software_target_factor`, `software_headroom_rpm` | Python overspeed threshold = min(max command × max-command factor, max(software minimum, target magnitude × target factor + headroom)). M0 uses 20 RPM, 1.12, 1.2, and 10 RPM. |
| `sample_seconds`, `count_min_interval_s` | Host polling and minimum count measurement interval, in seconds. M0 uses 0.07 s and 0.012 s. The shared `runtime.display_every_s` only limits terminal output. |
| `safety`, `calibration`, `runtime` | Per-axis supply/operating voltage windows, maximum permitted motor current, stationary-speed threshold, matching tolerances, firmware watchdog timeout, and calibration acceptance criteria. The ranges, state deadlines, and polling intervals use the units in their field names. |

In encoder velocity, Python stops on the first count-speed sample above the
software threshold. The estimated speed must exceed the software threshold
for 0.15 s of observed elapsed time, or cross the immediate limit, whichever
happens first. The immediate limit is the lower of 125% of the software
threshold and the firmware trip RPM. Sensorless retains the original immediate
estimator cutoff at the software threshold. The encoder stall check starts
at 20 RPM or 90% of the target, whichever is lower. It stops the trial if
count speed stays below 25% of setpoint in the commanded direction for 0.5 s.

## Tuning and reading logs

1. Verify wiring, supply, brake resistor, pole pairs, and CPR. Use the raw count
   monitor for a hand-turned revolution while IDLE.
2. Set current and calibration limits for the installed motor. Run calibration
   and confirm plausible R/L and encoder scan measurements.
3. Start with a short unloaded encoder-velocity trial at an allowed low RPM.
   Check direction, setpoint, estimated and count-derived speeds, current, and
   bus voltage. Tune the velocity PI gains and ramp one value at a time.
   Change inner current-loop bandwidth only when you need to tune that loop.
4. If feedback collapses or runs backward under command, inspect the axis fault
   codes and repeat wiring/CPR and alignment checks before another powered trial.
   If IDLE cannot be confirmed, isolate motor power.

Each velocity trial writes `logs/m<axis>_<mode>_<UTC time>_<signed target>rpm.csv`.
The `start` row contains the configuration and trial settings. `sample` rows
contain UTC time, elapsed seconds, axis, target and setpoint RPM, estimated
and count-derived RPM, accumulated A/B counts, electrical phase where available,
Iq setpoint, measured Iq and Id, bus voltage, effective motor current limit, and raw
error masks. Encoder values are blank when unavailable. The `stop` row contains
the reason, details, final fault codes where readable, and whether IDLE was
confirmed. Stops are classified as `user_stop`, `overspeed`, `stall`, `fault`,
`timeout`, or `normal_completion`. Preflight refusals happen before a trial starts.

Run the mocked suite with `.venv/bin/python -m unittest discover -s tests -v`.
