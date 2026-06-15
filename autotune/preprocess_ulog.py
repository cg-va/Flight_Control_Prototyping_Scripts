#!/usr/bin/env python3
"""
preprocess_ulog.py — Filter a PX4 ulog for FCPS offline system identification.

Workflow:
    ulog → load → filter+decimate vehicle_torque_setpoint + vehicle_angular_velocity
         → write new ulog → user loads in unchanged FCPS GUI (autotune.py)

Recipe (locked, see Testing_Auswertungen/HANDOFF_real_flight_filtering_research.md):
  • Notch filter at hover-motor-fundamental (RPM × N_blades / 60), Q=4
      - RPM computed live from esc_status.esc_rpm when present
      - Fallback to mode-specific default if ESC topic missing
  • Low-pass 4th-order Butterworth via scipy.signal.filtfilt (zero-phase)
      - 20 Hz default for MC mode, 15 Hz for FW mode
  • Decimate 4× via scipy.signal.decimate(zero_phase=True) — 400 Hz → 100 Hz
  • Drop first/last 1 s of window (filtfilt edge safety)
  • Prints informational metrics (coherence, band fraction before/after) and
    recommended FCPS GUI settings to stdout

Christian Güttler, Vortex Aerotec — 2026-05-21
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog
from scipy import signal as sp

# Recipe defaults
DEFAULTS = {
    'mc': {'lp_hz': 20.0, 'fallback_notch_hz': 78.0,  'esc_motor_indices': list(range(0, 8))},
    'fw': {'lp_hz': 15.0, 'fallback_notch_hz': 43.0,  'esc_motor_indices': [8, 9]},
}
LP_ORDER = 4
DECIMATE_FACTOR_DEFAULT = 4
NOTCH_Q = 4.0
EDGE_TRIM_S = 1.0
COH_BAND_HZ = (0.5, 10.0)


def get_topic(log, name, multi_id=0):
    for d in log.data_list:
        if d.name == name and d.multi_id == multi_id:
            return d
    return None


def compute_notch_freq(log, t_start, t_end, mode, n_blades):
    esc = get_topic(log, 'esc_status')
    if esc is None:
        return DEFAULTS[mode]['fallback_notch_hz'], 'fallback (esc_status not in ulog)'
    t = np.array(esc.data['timestamp']) * 1e-6
    in_window = (t >= t_start) & (t <= t_end)
    if not in_window.any():
        return DEFAULTS[mode]['fallback_notch_hz'], 'fallback (no esc_status samples in window)'
    rpms = []
    for i in DEFAULTS[mode]['esc_motor_indices']:
        key = f'esc[{i}].esc_rpm'
        if key in esc.data:
            rpm = np.array(esc.data[key])[in_window]
            rpm = rpm[rpm > 100]  # exclude zeros (motor not running)
            if len(rpm) > 0:
                rpms.append(float(rpm.mean()))
    if not rpms:
        return DEFAULTS[mode]['fallback_notch_hz'], 'fallback (all rpm < 100 in window)'
    avg_rpm = float(np.mean(rpms))
    notch_hz = avg_rpm * n_blades / 60.0
    return notch_hz, f'live from {len(rpms)} motors, mean RPM {avg_rpm:.0f}, {n_blades}-blade'


def filter_array(arr, fs, notch_hz, lp_hz, lp_order, decimate_factor):
    """filtfilt(notch) → filtfilt(LP) → decimate. Returns filtered+decimated array."""
    out = np.asarray(arr, dtype=np.float64)
    if notch_hz is not None and notch_hz > 0 and notch_hz < fs/2:
        b, a = sp.iirnotch(notch_hz, NOTCH_Q, fs)
        out = sp.filtfilt(b, a, out)
    if lp_hz < fs/2:
        b, a = sp.butter(lp_order, lp_hz / (fs/2), btype='low')
        out = sp.filtfilt(b, a, out)
    if decimate_factor and decimate_factor > 1:
        out = sp.decimate(out, decimate_factor, ftype='iir', zero_phase=True)
    return out


def metrics(u, y, fs, f_lo=COH_BAND_HZ[0], f_hi=COH_BAND_HZ[1]):
    """Mean coherence + excitation-band energy fraction (informational; no gate)."""
    n = min(2048, max(64, len(u) // 8))
    f, cxy = sp.coherence(u, y, fs=fs, nperseg=n)
    band = (f >= f_lo) & (f <= f_hi)
    coh_mean = float(cxy[band].mean()) if band.any() else float('nan')
    _, Puu = sp.welch(u, fs=fs, nperseg=n)
    total = float(np.trapz(Puu, f))
    band_pow = float(np.trapz(Puu[band], f[band])) if band.any() else 0.0
    band_frac = band_pow / total if total > 0 else float('nan')
    return coh_mean, band_frac


def filter_topic_inplace(topic, t_start, t_end, notch_hz, lp_hz, decimate_factor, edge_trim_s):
    """
    Trim a Data object to the identification window, filter+decimate all numeric fields,
    rewrite timestamps. Returns (orig_fs, new_fs, metrics_per_axis).
    """
    t = np.array(topic.data['timestamp']) * 1e-6
    mask = (t >= t_start) & (t <= t_end)
    if mask.sum() < 100:
        raise ValueError(f'Topic {topic.name} has only {mask.sum()} samples in window')

    t_win = t[mask]
    dt_orig = float(np.median(np.diff(t_win)))
    fs_orig = 1.0 / dt_orig

    # Process xyz[0/1/2] with filter; other numeric fields with decimate-only
    new_data = {}
    metrics_per_axis = {}
    for field, arr in topic.data.items():
        arr_w = np.asarray(arr)[mask]
        if field.startswith('xyz['):
            filt = filter_array(arr_w, fs_orig, notch_hz, lp_hz, LP_ORDER, decimate_factor)
            new_data[field] = filt.astype(arr.dtype if arr.dtype != np.float64 else np.float32)
        elif arr.dtype.kind in ('i', 'u'):
            if decimate_factor and decimate_factor > 1:
                new_data[field] = arr_w[::decimate_factor]
            else:
                new_data[field] = arr_w.copy()
        else:
            if decimate_factor and decimate_factor > 1:
                dec = sp.decimate(arr_w.astype(np.float64), decimate_factor,
                                  ftype='iir', zero_phase=True)
                new_data[field] = dec.astype(arr.dtype)
            else:
                new_data[field] = arr_w.copy()

    # Length consistency: filter+decimate produces a specific length; force everything to match
    target_len = len(new_data[next(k for k in new_data if k.startswith('xyz['))])
    for k in list(new_data.keys()):
        if len(new_data[k]) != target_len:
            if len(new_data[k]) > target_len:
                new_data[k] = new_data[k][:target_len]
            else:
                # pad by repeating last value
                pad = target_len - len(new_data[k])
                new_data[k] = np.concatenate([new_data[k], np.full(pad, new_data[k][-1], dtype=new_data[k].dtype)])

    # Edge trim
    fs_new = fs_orig / max(1, decimate_factor or 1)
    trim_samples = int(edge_trim_s * fs_new)
    if trim_samples > 0 and target_len > 2 * trim_samples:
        for k in list(new_data.keys()):
            new_data[k] = new_data[k][trim_samples:-trim_samples]
        target_len -= 2 * trim_samples

    # Preserve original timestamps (don't rebuild as uniform — that breaks per-axis window
    # extraction on logs with irregular sampling, e.g. PX4 autotune logs where the chirp
    # window can drop from 400 Hz to ~287 Hz under CPU load. Rebuilding as uniform at the
    # median rate compresses time-space and causes cross-axis sample contamination.
    # Reuse the timestamps from the original window, trimmed + decimated identically.
    orig_ts = np.asarray(topic.data['timestamp'])[mask]
    if decimate_factor and decimate_factor > 1:
        orig_ts = orig_ts[::decimate_factor]
    # length-match orig_ts to target post-trim length
    if trim_samples > 0 and len(orig_ts) >= 2 * trim_samples:
        orig_ts = orig_ts[trim_samples:len(orig_ts) - trim_samples]
    # Pad/truncate to match target_len (rounding/decimate-tail can introduce off-by-one)
    if len(orig_ts) > target_len:
        orig_ts = orig_ts[:target_len]
    elif len(orig_ts) < target_len:
        # extend by extrapolating last delta
        if len(orig_ts) >= 2:
            dt_us = orig_ts[-1] - orig_ts[-2]
            pad = orig_ts[-1] + np.arange(1, target_len - len(orig_ts) + 1, dtype=orig_ts.dtype) * dt_us
            orig_ts = np.concatenate([orig_ts, pad])
        else:
            orig_ts = np.concatenate([orig_ts, np.full(target_len - len(orig_ts), orig_ts[-1] if len(orig_ts) else 0, dtype=orig_ts.dtype)])
    new_data['timestamp'] = orig_ts.astype(np.uint64)
    if 'timestamp_sample' in new_data:
        # use original timestamp_sample if present, same processing
        orig_tss = np.asarray(topic.data['timestamp_sample'])[mask]
        if decimate_factor and decimate_factor > 1:
            orig_tss = orig_tss[::decimate_factor]
        if trim_samples > 0 and len(orig_tss) >= 2 * trim_samples:
            orig_tss = orig_tss[trim_samples:len(orig_tss) - trim_samples]
        if len(orig_tss) > target_len:
            orig_tss = orig_tss[:target_len]
        elif len(orig_tss) < target_len:
            pad_len = target_len - len(orig_tss)
            orig_tss = np.concatenate([orig_tss, np.full(pad_len, orig_tss[-1] if len(orig_tss) else 0, dtype=orig_tss.dtype)])
        new_data['timestamp_sample'] = orig_tss.astype(np.uint64)

    # Compute axis metrics on the filtered data (only valid for vehicle_torque_setpoint + vehicle_angular_velocity)
    if topic.name == 'vehicle_torque_setpoint':
        for ax in range(3):
            metrics_per_axis[f'axis{ax}_input_std'] = float(new_data[f'xyz[{ax}]'].std())

    topic.data = new_data
    return fs_orig, fs_new, metrics_per_axis


def main():
    ap = argparse.ArgumentParser(
        description='Filter ulog for FCPS offline system ID',
        epilog='Output is a valid ulog the unchanged FCPS GUI loads as-is.',
    )
    ap.add_argument('input', type=Path, help='Input ulog path')
    ap.add_argument('output', type=Path, help='Output filtered ulog path')
    ap.add_argument('--mode', choices=['mc', 'fw'], default='mc',
                    help='Flight mode (sets LP cutoff + ESC motor selection)')
    ap.add_argument('--lp-hz', type=float, default=None,
                    help='LP cutoff override (default: 20 Hz for mc, 15 Hz for fw)')
    ap.add_argument('--notch-hz', type=float, default=None,
                    help='Notch override Hz (default: live from esc_status)')
    ap.add_argument('--n-blades', type=int, default=2, help='Blades per prop (default 2)')
    ap.add_argument('--decimate', type=int, default=DECIMATE_FACTOR_DEFAULT,
                    help='Decimation factor (default 4)')
    ap.add_argument('--no-decimate', action='store_true')
    ap.add_argument('--no-notch', action='store_true')
    ap.add_argument('--chirp', action='store_true',
                    help='Chirp-mode preset for PX4 autotune logs (short ~5s per-axis windows): '
                         'just filtfilt LP at --lp-hz, no notch, no decimate, edge-trim 0.2s. '
                         'Empirically wins on short chirp windows where decimation kills RLS convergence.')
    ap.add_argument('--t-start', type=float, default=None,
                    help='Window start in s after arming (default: 0)')
    ap.add_argument('--t-end', type=float, default=None,
                    help='Window end in s after arming (default: disarm)')
    args = ap.parse_args()

    if args.chirp:
        # Chirp-mode preset: short per-axis windows from PX4 autotune.
        # Aggressive decimation + notch hurts more than helps when window is 5-10s.
        # Empirical result on real-flight log_42: chirp mode jumps fit from ~60% raw
        # to ~70% per axis; full preprocessor drops it to 20-55%.
        args.no_notch = True
        args.no_decimate = True
        if args.lp_hz is None:
            args.lp_hz = 20.0  # MC default; user can override

    if not args.input.exists():
        sys.exit(f'Input not found: {args.input}')

    print(f'Loading {args.input.name}...')
    log = ULog(str(args.input))

    vstatus = get_topic(log, 'vehicle_status')
    if vstatus is None:
        sys.exit('Cannot find vehicle_status in ulog — cannot determine arming window')
    arm_t = np.array(vstatus.data['timestamp']) * 1e-6
    arm_state = np.array(vstatus.data['arming_state'])
    armed_t = arm_t[arm_state == 2]
    if len(armed_t) == 0:
        sys.exit('No armed samples in ulog')
    t0_arm = float(armed_t[0])
    t_disarm = float(armed_t[-1])
    t_start = t0_arm + (args.t_start if args.t_start is not None else 0)
    t_end = (t0_arm + args.t_end) if args.t_end is not None else t_disarm
    print(f'Identification window: {t_start - t0_arm:.1f}..{t_end - t0_arm:.1f}s after arm ({t_end - t_start:.1f}s total)')

    lp_hz = args.lp_hz if args.lp_hz is not None else DEFAULTS[args.mode]['lp_hz']
    decimate_factor = None if args.no_decimate else args.decimate

    if args.no_notch:
        notch_hz, notch_src = None, 'disabled by --no-notch'
    elif args.notch_hz is not None:
        notch_hz, notch_src = args.notch_hz, f'user override {args.notch_hz} Hz'
    else:
        notch_hz, notch_src = compute_notch_freq(log, t_start, t_end, args.mode, args.n_blades)

    print(f'Notch:  {notch_hz} Hz ({notch_src})')
    print(f'LP:     {lp_hz} Hz, 4th-order Butterworth filtfilt')
    print(f'Decim:  {decimate_factor or "off"}')
    print()

    # Metrics BEFORE preprocessing for sidecar comparison
    ts_topic = get_topic(log, 'vehicle_torque_setpoint')
    av_topic = get_topic(log, 'vehicle_angular_velocity')
    if ts_topic is None or av_topic is None:
        sys.exit('vehicle_torque_setpoint or vehicle_angular_velocity missing from ulog')

    t_ts = np.array(ts_topic.data['timestamp']) * 1e-6
    mask_ts = (t_ts >= t_start) & (t_ts <= t_end)
    fs_orig_ts = 1.0 / float(np.median(np.diff(t_ts[mask_ts])))
    metrics_before = {}
    for ax in range(3):
        u_raw = np.array(ts_topic.data[f'xyz[{ax}]'])[mask_ts]
        t_av = np.array(av_topic.data['timestamp']) * 1e-6
        av_raw = np.interp(t_ts[mask_ts],
                           t_av[(t_av >= t_start) & (t_av <= t_end)],
                           np.array(av_topic.data[f'xyz[{ax}]'])[(t_av >= t_start) & (t_av <= t_end)])
        coh, band_frac = metrics(u_raw, av_raw, fs_orig_ts)
        metrics_before[f'axis{ax}'] = {'coherence_0p5_10hz': round(coh, 3), 'band_excitation_frac': round(band_frac, 4)}

    # Chirp mode uses a shorter edge trim because per-axis chirp windows are typically 5-10s;
    # a 1s trim on each side burns 20-40% of the window. 0.2s is enough for filtfilt boundary safety.
    edge_trim_s = 0.2 if args.chirp else EDGE_TRIM_S

    print('Filtering vehicle_torque_setpoint...')
    fs_orig_ts, fs_new_ts, _ = filter_topic_inplace(ts_topic, t_start, t_end, notch_hz, lp_hz, decimate_factor, edge_trim_s)
    print(f'  {fs_orig_ts:.0f} Hz -> {fs_new_ts:.0f} Hz, {len(ts_topic.data["timestamp"])} samples')

    print('Filtering vehicle_angular_velocity...')
    fs_orig_av, fs_new_av, _ = filter_topic_inplace(av_topic, t_start, t_end, notch_hz, lp_hz, decimate_factor, edge_trim_s)
    print(f'  {fs_orig_av:.0f} Hz -> {fs_new_av:.0f} Hz, {len(av_topic.data["timestamp"])} samples')

    # Metrics AFTER (compute from the new filtered topic data directly)
    metrics_after = {}
    for ax in range(3):
        u = ts_topic.data[f'xyz[{ax}]']
        # Resample av onto ts timestamps (now identical rate if both decimated by same factor)
        if len(u) == len(av_topic.data[f'xyz[{ax}]']):
            y = av_topic.data[f'xyz[{ax}]']
        else:
            t_ts_new = ts_topic.data['timestamp'] * 1e-6
            t_av_new = av_topic.data['timestamp'] * 1e-6
            y = np.interp(t_ts_new, t_av_new, av_topic.data[f'xyz[{ax}]'])
        coh, band_frac = metrics(u, y, fs_new_ts)
        metrics_after[f'axis{ax}'] = {'coherence_0p5_10hz': round(coh, 3), 'band_excitation_frac': round(band_frac, 4)}

    print(f'\nWriting {args.output.name}...')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log.write_ulog(str(args.output))

    print('\n=== Metrics summary (informational, no gating) ===')
    print(f'{"axis":6s}  {"before coh":>11s} {"after coh":>10s}  {"before band":>12s} {"after band":>11s}')
    for ax in range(3):
        b = metrics_before[f'axis{ax}']; a = metrics_after[f'axis{ax}']
        print(f'  axis{ax}  {b["coherence_0p5_10hz"]:11.3f} {a["coherence_0p5_10hz"]:10.3f}  '
              f'{b["band_excitation_frac"]:12.4f} {a["band_excitation_frac"]:11.4f}')

    # FCPS GUI recommended settings — empirically delays 2-3 at 100 Hz captures
    # physical plant delay (~10-30 ms rotor/ESC lag) + residual filter group delay
    # from decimation's anti-alias. "Find parameters" will refine.
    print()
    print('=== FCPS GUI recommended settings ===')
    print(f'  HP cutoff: 0.5 Hz (FCPS default — keep)')
    print(f'  LP cutoff: 30 Hz  (FCPS default — keep; preprocessor already band-limited to {lp_hz} Hz)')
    print(f'  Zeros / Poles / Delays:  2 / 2 / 2  (Find parameters will refine; delays 2-3 typical post-decimation)')
    print(f'  Method:    RLS')
    print()
    print(f'Done. Open {args.output} in FCPS autotune.py GUI.')


if __name__ == '__main__':
    main()
