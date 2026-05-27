# Installation using a virtual environment

## Using Poetry (recommended)
### Create a new environment and install the dependencies
1. Get [poetry](https://python-poetry.org/docs/): `curl -sSL https://install.python-poetry.org | python3 -`
2. In this directory, simply run `poetry install`

### Start the GUI
```
poetry run python3 autotune.py
```

## Using venv
### Create a new environment
```
python3.9 -m venv virtualenv-test
source virtualenv-test/bin/activate
```

### Install the dependencies
```
pip3 install numpy scipy pyulog control pyqt5
```

### Start the GUI
```
python3 autotune.py
```

![image](https://github.com/user-attachments/assets/fcdf5c25-d92d-4487-9736-e77f6576d180)

# Tuning Worflow

## Fixed-Wing Rate Tuning

**Prerequisites**:

- Requires valid airspeed measurement on vehicle
- Correctly set FW_AIRSPD_TRIM

### Flight Maneuvers

Ensure your vehicle passes the pre-tuning test as outlined [here](https://docs.px4.io/main/en/config/autotune_fw.html#pre-tuning-test).

#### Use automated maneuvers (Recommended)

1. Configure your setup to use the PX4 autotune maneuvers without applying the identified gains online:

```
   FW_AT_APPLY = Do not apply the new gains (logging only) 
   FW_AT_AXES = 7  # Enables maneuvers on all three axes (adjust as needed) 
   FW_AT_SYSID_F1 = 10
   FW_AT_SYSID_TYPE = Logarithmic sine sweep
```

2. Enable an [autotune switch](https://docs.px4.io/main/en/config/autotune_fw.html#enable-disable-autotune-switch) on your RC transmitter. 
3. Follow the procedure described [here](https://docs.px4.io/main/en/config/autotune_fw.html#auto-tuning-procedure) to perform the maneuvers.
Since FW_AT_APPLY is set to logging only, step 5 of the procedure (applying new gains and testing) will not be executed.

#### Perform manual maneuvers

Perform these maneuvers separately for each axis: roll, pitch, and yaw.

1. Fly in stabilized mode and start from level flight.
2. Apply a manual sine chirp input (sine wave with increasing frequency) on the selected axis. Avoid any other control inputs simultaneously.
3. For pitch and yaw: The amplitude of the input should not be too high, but should be enough to overcome noise.
4. Repeat the maneuver a few times if necessary to capture good data.

### Using the Tool

#### Load the flight log and select the relevant tuning loop (roll, pitch, or yaw):

![image](https://github.com/user-attachments/assets/f962a004-eae8-433f-9b7e-adc618dcbeae)

#### Window selection:

1. Select the portion of the log where the maneuver occurred.
2. Ensure the window starts from level flight and covers the full maneuver.
3. (Experimental) Use the coherence plot to check whether your input-output data have a strong enough linear relationship at the desired frequencies. For fixed-wings: \
   i. We are interested in frequencies between 0.5-10Hz. \
   ii. Values > 0.6 are sufficient. \
   iii. If the coherence function is oscillating dramatically, the identified model may be unreliable at those frequencies.
4. Once you have identified the window, load the selection into the tool.

<table>
  <tr>
    <td align="center">
      <b>Good window selection</b><br>
      <img src="https://github.com/user-attachments/assets/25c30295-6825-4cb4-828e-47eaa17e1557" width="400"/><br>
      <i>Starts from level flight and covers the full maneuver. Coherence is sufficient.</i>
    </td>
    <td align="center">
      <b>Bad window selection</b><br>
      <img src="https://github.com/user-attachments/assets/1e05d411-37cf-42ab-8d6f-0cbbbb00921c" width="400"/><br>
      <i>Does not isolate the maneuver. Coherence is insufficient. </i>
    </td>
  </tr>
</table>

#### Model Verification

1. Check that the estimated dynamic model correctly tracks the actual aircraft output.
2. If not, adjust the system order (number of poles and zeros) and/or delay parameters.
3. Verify that the system is minimum-phase (i.e., zeroes lie inside the unit circle).

<img src="https://github.com/user-attachments/assets/87168b77-1b52-4d6f-bb4d-ace3f76fe4ee" width="700"/>

#### Tuning Gains

1. Set all gains to zero
2. Increase the P-gain until the step response shows oscillations. Then reduce the P-gain by approximately 50%:

   <b>Increase P-gain</b><br>
   <img src="https://github.com/user-attachments/assets/db05a018-f9fb-4b3c-9f55-a4d7a577251f"/>

   <b>Decrease by 50%</b><br>
   <img src="https://github.com/user-attachments/assets/77dc38aa-f829-4878-a58e-32d775fa542c"/>

3. Adjust the I-gain to achieve a good step response with acceptable steady-state error.

   ![image](https://github.com/user-attachments/assets/78f3bca5-2642-44e2-b3f2-7444fc4ffa40)

4. Fine-tune both gains as needed. At the 1 second mark, a disturbance is injected into the system. The tuned response to this disturbance is visible in the step response plot.

5. Use the Bode plot to confirm that the resonant frequency peak remains below 0dB.

   ![image](https://github.com/user-attachments/assets/86d99295-43a8-47a9-b052-324ab5ac3082)

### Applying the Gains

1. The gains shown in the parallel form correspond to the PX4 rate controller parameters.
   ![image](https://github.com/user-attachments/assets/2db77238-bd33-4454-af1f-a38901571e88)

2. If the identified parameters are strongly different to the currently set ones (more than 30% for multiple axes), an incremental application while flying is recommended (not prior to takeoff, during takeoff, or during landing).

3. For all loops tuned with the strategy outlined in this guide: increase the respective integrator limit parameters to 1 (FW_RR_IMAX, FW_PR_IMAX, FW_YR_IMAX).

---

> Everything below this divider is **Vortex-Aerotec additions** beyond upstream Auterion. 

## Pre-processing parameters explained — HP, LP, Input scaling

The three knobs in the GUI's "Pre-processing" group are easy to misuse. Here's what each does and when it matters.

### HP cutoff (Hz) — default 0.5

First-order forward-only IIR high-pass applied to input `u` and output `y` before ARX RLS. Removes DC offset, trim, and slow drift. **Keep at 0.5 Hz default in almost all cases.**

- If too low (e.g. 0.01 Hz): trim offsets and slow drift contaminate the fit. The 5-coefficient ARX(2,2) spends degrees of freedom modeling the offset instead of the dynamics. Bad fit.
- If too high (e.g. 5 Hz): you start killing the low-frequency content of the plant itself (rate dynamics live in roughly 1–10 Hz). Also bad.
- Sweet spot is 0.3–0.7 Hz. We've stuck with 0.5 Hz on every campaign.

### LP cutoff (Hz) — default 30

First-order forward-only IIR low-pass applied to `u` and `y`. **Keep at 30 Hz default. Do NOT lower this to clean up noisy real-flight data.**

Why not? Forward-only IIR introduces phase lag (group delay) that grows as the cutoff drops. ARX(n, m, d) interprets that group delay as plant transport delay and burns the `delays` parameter on it. 

If you actually need aggressive low-pass filtering (real-flight data with prop / vibration noise above the band of interest), use the **preprocessor script** described below.

### Input scaling 

Three options in the dropdown:

| Option | When to use |
|---|---|
| **None** | **REQUIRED for multicopter mode.** Hover airspeed ≈ 0 m/s; the airspeed-scaling options divide the input by airspeed, which blows up numerically near zero and corrupts the ARX fit. |
| **True airspeed** | Fixed-wing only. Scales input by `airspeed / trim_airspeed`, useful if you want a single linear model valid across part of the cruise envelope. |
| **True airspeed²** | Fixed-wing only. Scales by `(airspeed / trim_airspeed)²` — proper dynamic-pressure normalization. The right choice if you're identifying a plant valid across a wide cruise band where control effectiveness scales with `q = ½ρv²`. | Use this most of the time.

**Attention:** the GUI auto-fills the `Trim airspeed` field from `FW_AIRSPD_TRIM` in any log that has it — including MC-mode logs from a VTOL airframe. Don't be fooled by the populated value: if you're identifying MC, make absolutely sure the dropdown is set to **None** before clicking "Load selection".

For VTOL airframes specifically: when you choose your preset (`Rollrate(FW)` vs `Rollrate`), update the input-scaling dropdown to match:

| Preset choice | Input scaling |
|---|---|
| `Rollrate`, `Pitchrate`, `Yawrate` (MC) | **None** |
| `Rollrate(FW)`, `Pitchrate(FW)`, `Yawrate(FW)` | **True airspeed** or **True airspeed²** (your call based on cruise envelope width) |
| `Rollrate(closed-loop)` etc. | Mode-dependent — same rule as above |

Make this part of your muscle memory when opening a VTOL log: **preset → input scaling → window**, in that order.

---

## Multicopter Rate Tuning

The upstream README only documents fixed-wing rate tuning. The same `autotune.py` works for multicopter rate identification with the right preset choice; PX4's MC autotune publishes the same `autotune_attitude_control_status` state machine that `data_extractor` already understands.

### Prerequisites for MC

- **PX4 MC autotune module enabled** in the airframe: `MC_AT_EN = 1`
- **Recommended apply mode for offline analysis**: `MC_AT_APPLY = 0` (log only — don't let PX4 commit suggested gains). Without this, you're letting PX4's online autotune apply its own (often pathological) gains alongside collecting data, which contaminates the comparison.
- A vehicle that's already stable enough to hover with current gains (per the PX4 pre-tuning test).

### Flight maneuvers — MC mode

PX4's MC autotune fires sign-alternating step inputs (not a logarithmic sine sweep — different from FW). The MC autotune param surface is much smaller than FW: there's no `MC_AT_AXES`, no `MC_AT_SYSID_TYPE`, no `MC_AT_SYSID_F0/F1`. The axis sequence (roll → pitch → yaw) is hard-coded; only `MC_AT_SYSID_AMP` (default 0.7) is user-tunable.

In `pxh`:
```
param set MC_AT_EN 1
param set MC_AT_APPLY 0
param set MC_AT_SYSID_AMP 0.7
param set MC_AT_RISE_TIME 0.14
param save
reboot
```

Take off in Altitude mode, hover at 5–10 m, then:
```
param set MC_AT_START 1
```

Wait ~30–60 s for the sequential roll → pitch → yaw step excitation, then land + disarm. Pull the ulog.

> **Heads up for heavy airframes (>10 kg class):** PX4's MC autotune state machine often transitions to `STATE_FAIL (12)` rather than `STATE_COMPLETE (11)` on these vehicles. Fitness across axes typically lands in the 0.5–8% range (vs >85% threshold PX4 wants). This is a published failure mode of MC autotune's narrow-band step excitation on heavy plants. Don't trust any gains PX4 publishes from such a run. Run FCPS offline on the captured ulog and identify yourself.

### Using the tool — MC mode

For each of the three axes (roll, pitch, yaw):

1. **Open log** → pick the chirp ulog.
2. **Preset** → `Rollrate` / `Pitchrate` / `Yawrate` (the ones **without** the `(FW)` suffix).
3. **Input scaling** → **None**. Verify before proceeding (see "Input scaling" section above).
4. **Drag-select** the excitation segment on the top time-domain plot. State-machine timing is roughly:
   - Roll: 5–20 s window (`autotune_attitude_control_status.state == 2`)
   - Pitch: 5–20 s (`state == 4`)
   - Yaw: 5–20 s (`state == 6`)
5. **Load selection** → **Find parameters** to search ARX orders + delay.
6. Verify model: pole-zero map (zeros inside the unit circle = minimum-phase), NRMSE fit ≥ 70% (heavy airframes may only hit 45–60% on manual-step-impulse data; that's the excitation ceiling, not a model order issue).
7. **GMVC tab** → set rise time σ to 0.05–0.15 s for MC (faster than FW typically; yaw on heavy airframes may want ~0.2 s). δ damping 0.5–1.0, λ detune 0.5. Adjust until: overshoot < 20%, settling < 0.5 s, Bode resonant peak < 0 dB, GM > 6 dB, PM > 45°.
8. **Read off the Parallel-form gains** (right column in the PID tab) — these map directly to `MC_*RATE_P`, `MC_*RATE_I`, `MC_*RATE_D` (with `MC_*RATE_K = 1.0` locked).

### Applying MC gains

Lock `MC_*RATE_K = 1.0`. Map FCPS Parallel-form gains directly to the corresponding `_P / _I / _D` params. Increase `MC_*RATE_IMAX` to 1.0 per the same rule as FW.

For dramatic gain changes, stage incrementally on first real flight:

| Flight | `MC_YAWRATE_P` |
|---|---|
| 1 | 1.0 (≈2.4× old; conservative meaningful improvement) |
| 2 | 1.5 (≈3.5× old; only if Flight 1 clean) |
| 3 | 2.087 (full FCPS value; only if Flight 2 clean) |

Watch `rate_ctrl_status.yawspeed_integ` for IMAX-rail saturation; watch for slow oscillation under crosswind = integrator wind-up.

---

## Real-flight ulog preprocessing — `preprocess_ulog.py`

**Why:** Real-flight ulogs contain substantial high-frequency feedback content on `vehicle_torque_setpoint` that the rate controller injects into its own output (P-term reading noisy `rate_meas`, attitude loop integrating gyro noise, common-mode mechanical vibration from props). This content is **not** real broadband plant excitation — it's closed-loop coupling within one control cycle. Empirical signature: cross-correlation between high-pass-filtered input and output peaks at 0.89 at zero lag (vs SITL chirp where input precedes output by tens of milliseconds, indicating real plant excitation).

If FCPS sees this HF content and tries to fit ARX(2,2) coefficients to it, the model burns all 5 coefficients on the feedback confound and produces a useless fit. Empirical: raw FCPS on a real-flight log gave **−53%** fit; preprocessed (filtfilt 15 Hz + notch + decimate) gave **+50%** fit on the same data, same axis, same window. ~100 percentage points of fit recovered by preprocessing alone.

**SITL data does not need preprocessing.** SITL has no prop vibration, no real IMU noise, and the chirp signal is clean — the raw ulog goes straight into the GUI.

### Usage

```bash
# Real-flight or HITL ulog, MC mode (LP 20 Hz, hover-motor notch)
python3 preprocess_ulog.py input.ulg output.ulg --mode mc

# Real-flight FW mode (LP 15 Hz, pusher-motor notch)
python3 preprocess_ulog.py input.ulg output.ulg --mode fw --t-start 30 --t-end 90

# Disable specific filter elements if needed
python3 preprocess_ulog.py input.ulg output.ulg --mode mc --no-notch
python3 preprocess_ulog.py input.ulg output.ulg --mode mc --no-decimate

# Override notch frequency (if ESC telemetry unavailable or wrong N_blades)
python3 preprocess_ulog.py input.ulg output.ulg --mode mc --notch-hz 156 --n-blades 2
```

The output is a valid ulog you load into this `autotune.py` GUI **as-is** — the GUI doesn't know or care that the data was preprocessed.

### Filter recipe (state-of-the-art per Tischler/Remple + NASA Ames CIFER)

1. **Notch filter** at hover-motor blade-pass frequency, computed live from `mean(esc_status.esc_rpm) × N_blades / 60` with Q=4. Falls back to 78 Hz (MC) or 43 Hz (FW) if the ESC topic is missing.
2. **Low-pass** 4th-order Butterworth via `scipy.signal.filtfilt` (zero-phase forward+backward). 20 Hz default for MC, 15 Hz for FW. Stops short of the band of interest (1–10 Hz for the plant).
3. **Decimate** 4× via `scipy.signal.decimate(zero_phase=True)`. 400 Hz → 100 Hz effective sample rate. Helps RLS forgetting factor behave numerically.
4. Notch is applied **before** LP (notch removes prop tones; LP rolls off the residual).
5. First and last 1 s of the window are trimmed for filtfilt edge safety.

Why filtfilt and not the GUI's built-in LP at 15 Hz: see the "LP cutoff" section above — forward-only IIR adds phase lag that ARX miss-fits. filtfilt is zero-phase AND doubles the rolloff (effective 8th-order from a 4th-order filter, −173 dB/dec vs −86 dB/dec). At LP=15 Hz, 100 Hz noise content passes at 0.0001% via filtfilt vs 14.8% via the GUI's 1st-order forward-only LP.

### Output

Filtered ulog written to the given output path. Recommended FCPS GUI settings (HP, LP, ARX order, delays) and informational diagnostic metrics (coherence + band-fraction before/after preprocessing) print to `stdout` for you to log into your tuning record (see procedure section below).

---

## Standardized tuning procedure — analysis log template

To make campaigns reproducible (across team members, across time, across airframes), capture the following for every FCPS tuning campaign. Fill in one of these per axis per campaign. Suggested home for these records is the Vortex-internal `Testing_Auswertungen` repo alongside the procedure doc.

```markdown
# FCPS Campaign Record — {Airframe} {Axis} {Date}

## Source ulog
- Filename:     <e.g. mc_autotune_chirp_2026-05-18.ulg>
- Capture mode: SITL / HITL / real flight
- PX4 commit:   <SHA of the firmware that flew>
- Excitation:   PX4 MC autotune step / PX4 FW log sine sweep / manual chirp

## Identification window
- t_start (relative to arm): <e.g. 0.0 s>
- t_end:                     <e.g. 63.3 s>
- Reason for window choice:  <e.g. covers state==2 roll segment cleanly>

## Preprocessing (if any)
- Tool:       <preprocess_ulog.py vN / none>
- LP cutoff:  <e.g. 20 Hz>
- Notch:      <e.g. 136.26 Hz from esc_rpm mean 4088, 2-blade>
- Decimate:   <e.g. 4× → 100 Hz>
- Output ulog: <path>

## FCPS GUI settings
- Preset:        <Rollrate / Rollrate(FW) / ...>
- Input scaling: <None / True airspeed / True airspeed²>
- HP cutoff:     <Hz>
- LP cutoff:     <Hz>
- Method:        <RLS / OLS>
- Find parameters → ARX (n_poles, n_zeros, delays): <(2, 2, 1) etc.>

## Identified model
- a1, a2, ...:      <coefficients>
- b0, b1, b2, ...:  <coefficients>
- NRMSE fit:        <e.g. 89.3%>
- Stable:           <yes / no — and if you used "Stabilize", note that>

## Synthesized PID gains (parallel form for PX4)
- σ (rise time):    <e.g. 0.10 s>
- δ (damping):      <e.g. 0.7>
- λ (detune):       <e.g. 0.5>
- Kp_parallel:      <e.g. 0.5140>
- Ki_parallel:      <e.g. 0.4112>
- Kd_parallel:      <e.g. 0.0026>
- FF (if FW):       <e.g. 0.318>

## Closed-loop margins (from Bode)
- Gain margin: <e.g. 16.15 dB @ 6.3 Hz>
- Phase margin: <e.g. 67.3 deg @ 1.2 Hz>

## Validation
- SITL mission RMSE before / after:  <e.g. 5.31 / 3.95 deg/s (-26%)>
- ...

## Decision
- COMMIT / RESOLVED-NEGATIVE / ITERATE
- Reasoning:        <one-line>
- New airframe values: <MC_PITCHRATE_P 0.36 → 0.5140 etc.>
- Old values preserved as comments: <yes/no>

## Real-flight staging (if committing dramatic change)
- Flight 1: <param = staged_value>
- Flight 2: ...
- Watch for: <e.g. slow oscillation under crosswind>
```

The discipline of filling this in pays off the first time someone (you, six months from now; or Jan; or whoever's onboarding) needs to re-run or reason about the campaign.

