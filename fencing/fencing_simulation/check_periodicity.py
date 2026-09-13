#!/usr/bin/env python3
"""
Check whether 3D vehicle motion is periodic at steady state.

Approach:
1. Load simulation data
2. Extract steady-state portion (last 100s)
3. Compute FFT of each vehicle's target-relative position (ξ_i = x_i - x_target)
4. Report dominant frequencies and their power
5. Also check autocorrelation for periodicity
"""

import numpy as np
from pathlib import Path

# Load data
data = np.load(Path(__file__).parent / "data" / "simulate_3d.npz")
t = data['t']
x_traj = data['x_traj']  # (N, 3, T)
x_target_traj = data['x_target_traj']  # (3, T)
N = x_traj.shape[0]

# Target-relative positions
xi_traj = x_traj - x_target_traj.reshape(1, 3, -1)  # (N, 3, T)

# Steady state: last 100 seconds
dt = t[1] - t[0]
mask_steady = t > (t[-1] - 100)
t_ss = t[mask_steady]
xi_ss = xi_traj[:, :, mask_steady]
T_ss = len(t_ss)

print(f"Steady-state window: {t_ss[0]:.1f}s to {t_ss[-1]:.1f}s ({T_ss} samples, dt={dt:.3f}s)")

# ============================================================
# FFT analysis: find dominant frequencies
# ============================================================
print("\n=== FFT Analysis (Steady State) ===")
print("Dominant frequencies (excluding DC) for each vehicle's ξ coordinates:")

all_freqs = []
all_powers = []

for i in range(N):
    print(f"\nVehicle {i+1}:")
    for dim, dim_name in enumerate(['x', 'y', 'z']):
        signal = xi_ss[i, dim, :]
        fft = np.fft.rfft(signal)
        power = np.abs(fft) ** 2
        freqs = np.fft.rfftfreq(T_ss, d=dt)

        # Skip DC component
        power[0] = 0
        # Find top 3 peaks
        top_idx = np.argsort(power)[-3:][::-1]
        peaks = [(freqs[j], power[j]) for j in top_idx if power[j] > 1e-10]
        all_freqs.extend([f for f, _ in peaks])
        all_powers.extend([p for _, p in peaks])

        peak_str = ", ".join([f"{f:.4f} Hz (P={p:.2e})" for f, p in peaks])
        print(f"  {dim_name}: {peak_str}")

# ============================================================
# Overall spectral analysis
# ============================================================
print("\n=== Overall Spectrum ===")
# Average power spectrum across all vehicles and dimensions
avg_power = np.zeros(len(np.fft.rfftfreq(T_ss, d=dt)))
freqs = np.fft.rfftfreq(T_ss, d=dt)
for i in range(N):
    for dim in range(3):
        signal = xi_ss[i, dim, :]
        fft = np.fft.rfft(signal)
        avg_power += np.abs(fft) ** 2
avg_power /= (3 * N)

# Find dominant peaks in average spectrum
avg_power[0] = 0  # kill DC
top_peaks = np.argsort(avg_power)[-5:][::-1]
print("Top 5 frequencies in average power spectrum:")
for idx in top_peaks:
    print(f"  f = {freqs[idx]:.4f} Hz, P = {avg_power[idx]:.2e}, T = {1/freqs[idx]:.3f}s" if freqs[idx] > 0 else f"  f = 0 (DC)")

# ============================================================
# Periodicity check: autocorrelation
# ============================================================
print("\n=== Autocorrelation Periodicity Check ===")
# For each vehicle, compute autocorrelation of ||ξ_i(t)||
for i in range(N):
    r = np.linalg.norm(xi_ss[i], axis=0)  # distance from target
    r = r - np.mean(r)
    # Autocorrelation via FFT
    fft_r = np.fft.rfft(r, n=2*T_ss)
    acorr = np.fft.irfft(fft_r * np.conj(fft_r))[:T_ss]
    acorr = acorr / acorr[0]  # normalize

    # Find peaks in autocorrelation (excluding lag 0)
    # Look for significant peaks that indicate periodicity
    from scipy.signal import find_peaks
    peaks, props = find_peaks(acorr[1:], height=0.1, distance=10)
    if len(peaks) > 0:
        # Sort by peak height
        order = np.argsort(props['peak_heights'])[::-1]
        top_peaks = peaks[order[:3]]
        top_heights = props['peak_heights'][order[:3]]
        peak_str = ", ".join([f"lag={p} ({p*dt:.2f}s), height={h:.3f}" for p, h in zip(top_peaks, top_heights)])
        print(f"Vehicle {i+1}: top autocorrelation peaks → {peak_str}")
    else:
        print(f"Vehicle {i+1}: no significant autocorrelation peaks found")

# ============================================================
# Check if motion is approximately periodic with a common period
# ============================================================
print("\n=== Common Period Search ===")
# Look for a period T such that ξ_i(t+T) ≈ ξ_i(t) for all i
# Method: for a range of candidate periods, compute the RMS difference
# between ξ_i(t) and ξ_i(t+T) over the steady state

max_lag = T_ss // 2
lags = np.arange(10, max_lag, 5)  # test periods from 10*dt to max_lag*dt
best_period = None
best_error = float('inf')

for lag in lags:
    if lag >= T_ss:
        break
    # Compare first (T_ss - lag) samples with shifted version
    total_sq_error = 0
    count = 0
    for i in range(N):
        for dim in range(3):
            diff = xi_ss[i, dim, :T_ss-lag] - xi_ss[i, dim, lag:]
            total_sq_error += np.sum(diff ** 2)
            count += len(diff)
    rms_error = np.sqrt(total_sq_error / count)
    if rms_error < best_error:
        best_error = rms_error
        best_period = lag

best_period_s = best_period * dt
print(f"Best approximate period: {best_period_s:.3f}s (RMS error = {best_error:.4f})")
print(f"Corresponding frequency: {1/best_period_s:.4f} Hz")

# Also check: is the RMS error small relative to the typical ||ξ||?
typical_xi = np.sqrt(np.mean(np.sum(xi_ss ** 2, axis=1)))
print(f"Typical ||ξ|| magnitude: {typical_xi:.4f}")
print(f"Relative error: {best_error/typical_xi*100:.2f}%")

if best_error / typical_xi < 0.05:
    print("→ Motion appears PERIODIC (relative error < 5%)")
elif best_error / typical_xi < 0.15:
    print("→ Motion is QUASI-PERIODIC (relative error 5-15%)")
else:
    print("→ Motion is NOT clearly periodic (relative error > 15%)")