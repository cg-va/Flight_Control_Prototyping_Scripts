# Flight Control Prototyping Scripts — Vortex Aerotec fork

This is **Vortex Aerotec's fork** of [Auterion's `Flight_Control_Prototyping_Scripts`](https://github.com/Auterion/Flight_Control_Prototyping_Scripts) with modifications targeting our VENTUS S2 Canard-VTOL workflow. The upstream repo is a collection of Python prototyping scripts by Auterion mirroring PX4 algorithms — useful for offline tuning, plant identification, and algorithm experimentation without flashing firmware.

**Default tool for the team:** `autotune/` — offline ARX/RLS plant identification + GMVC PID synthesis from PX4 ulogs. See [autotune/README.md](autotune/README.md) for the full tuning procedure including FW + MC workflows, filter preprocessing, and the standardized analysis log template.

---

## What's different from upstream

| Change | File | Purpose |
|---|---|---|
| `preprocess_ulog.py` (NEW, preliminary) | [autotune/preprocess_ulog.py](autotune/preprocess_ulog.py) | Filters real-flight ulogs before they go into the unchanged `autotune.py` GUI. Adds zero-phase Butterworth LP, ESC-RPM-tracked notch, decimation. Required to get usable identification from real-flight data; SITL chirps generally don't need it. |
| Locale-safe trim-airspeed read | [autotune/autotune.py](autotune/autotune.py) | Replaces `float(line_edit_trim.text())` with `line_edit_trim.value()` so de_AT / de_DE / fr / etc. locales (decimal-comma) don't crash on log open. Plus a guard against `t_est` access before identification. |
| German / VTOL-flavored README additions | [autotune/README.md](autotune/README.md) | New sections on MC mode workflow, what the HP/LP/Input-scaling knobs actually do, when to use the preprocessor, and the standardized per-campaign analysis log template. |
| Repo-level `.gitignore` | [.gitignore](.gitignore) | Standard Python hygiene + `*.log` / `*.ulg` exclusion under specific paths. |

We pull from upstream `origin` (Auterion) regularly via merge. Christian works on the `christian` branch; team uses that branch directly. **Do not PR to `origin/master` unless explicitly approved** — these changes are Vortex-internal.

---

## Repository tour

Each subfolder is a standalone prototyping module — they don't depend on each other. Pick the one matching the algorithm or analysis you're doing.

| Folder | Purpose | Vortex use |
|---|---|---|
| **[autotune/](autotune/)** | Offline ARX/RLS plant ID + GMVC PID synthesis. Reads a PX4 ulog containing a chirp / step excitation, identifies the rate-controller plant model, synthesizes PID gains. **This is the team's primary tool.** | See `autotune/README.md` for the full workflow including our preprocessor + the procedure template. |
| **[control_allocation/](control_allocation/)** | Symbolic + numerical exploration of the PX4 control allocation strategy (saturation minimization via the orthogonal-vector approach, alternative to iterative re-mixing). | Reference for understanding PX4's allocation behavior.|
| **[drag_fusion_tuning/](drag_fusion_tuning/)** | Tunes EKF2 drag-fusion parameters (`EKF2_MCOEF`, `EKF2_BCOEF_X`, `EKF2_BCOEF_Y`) for wind estimation on multirotors from flight log data.|
| **[filters/](filters/)** | Quick comparison plot for different digital filter designs (Butterworth, etc.). | Reference; |
| **[hover_thrust_estimator/](hover_thrust_estimator/)** | Hover-thrust estimator replay + simulation. Validates the PX4 algorithm offline against a flight log. 
| **[leaky_integrator/](leaky_integrator/)** | Standalone test for the leaky-integrator `y[n] = α·y[n-1] + (α-1)·x[n]` filter shape used in various PX4 modules. | Reference. |
| **[quaternion_attitude_control/](quaternion_attitude_control/)** | Standalone test for the quaternion-based attitude controller algorithm used in PX4. | 
| **[range_finder_kinematic_consistency/](range_finder_kinematic_consistency/)** | Replays a flight log to check the consistency of range-finder altitude estimates against IMU integration. 
| **[trajectory_generator/](trajectory_generator/)** | Closed-loop Ziegler-Nichols tuning for the position controller; velocity trajectory generation reference. | Reference for understanding the position-control side. 

---

## Setup

```bash
# clone (assuming you've forked or have access)
git clone https://github.com/cg-va/Flight_Control_Prototyping_Scripts.git
cd Flight_Control_Prototyping_Scripts

# install python dependencies (Poetry preferred; venv works too)
curl -sSL https://install.python-poetry.org | python3 -
export PATH="$HOME/.local/bin:$PATH"
cd autotune
poetry install
poetry run python3 autotune.py    # smoke test
```

Make sure to clone from christian branch (initially)

---

## Where to find more

- **Full tuning procedure** (FW + MC, preprocessor, GMVC synthesis, validation gates): [autotune/README.md](autotune/README.md)

