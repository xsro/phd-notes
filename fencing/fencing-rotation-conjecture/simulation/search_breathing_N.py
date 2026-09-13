"""
Efficient search for breathing limit cycles with N ≤ 15 vehicles.

Uses shorter integration but more initial conditions.
Runs individual N values for parallel execution.
"""

import sys, os, json
import numpy as np
from scipy.signal import find_peaks

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fencing_ode import integrate, evaluate

D, MU, K1, K2 = 5.0, 9.0, 0.5, 0.5
TF = 400.0
T_SKIP = 150.0
DT = 0.2
TRAJECTORY_DT = 0.01  # fine step for saved trajectory data


def make_regular_polygon(N, R0=5.0, pert_pos=0.0, pert_vel=0.0, seed=42):
    rng = np.random.RandomState(seed)
    angles = np.linspace(0, 2*np.pi, N, endpoint=False)
    XI0 = R0 * np.column_stack([np.cos(angles), np.sin(angles)])
    if pert_pos > 0:
        XI0 += pert_pos * rng.randn(N, 2)
    norms = np.linalg.norm(XI0, axis=1, keepdims=True) + 1e-8
    VT0 = np.sqrt(K2) * np.column_stack([-XI0[:, 1], XI0[:, 0]]) / norms
    if pert_vel > 0:
        VT0 += pert_vel * rng.randn(N, 2)
    return XI0, VT0


def make_chaotic_config(N, R0=6.0, seed=42):
    rng = np.random.RandomState(seed)
    angles = rng.uniform(0, 2*np.pi, N)
    radii = rng.uniform(0.3*R0, 1.5*R0, N)
    XI0 = np.column_stack([radii * np.cos(angles), radii * np.sin(angles)])
    VT0 = rng.randn(N, 2) * 1.5
    return XI0, VT0


def analyze_trajectory(k1, k2, d, mu, xi0, vt0, tf=TF, t_skip=T_SKIP,
                     strategy=None, trajectory_dir=None):
    try:
        rhs, sol = integrate(xi0, vt0, d, mu, k1, k2, tf,
                             method="BDF", rtol=1e-7, atol=1e-9)
    except Exception as e:
        return {'error': str(e), 'is_breathing': False}

    # Fine-grid trajectory for saving (step = TRAJECTORY_DT)
    if trajectory_dir is not None and strategy is not None:
        t_fine = np.arange(t_skip, tf + TRAJECTORY_DT, TRAJECTORY_DT)
        if len(t_fine) > 0:
            pos_fine, vel_fine = evaluate(sol, len(xi0), t_fine)
            traj_path = save_trajectory(
                pos_fine, vel_fine, t_fine, len(xi0),
                k1, k2, d, mu, strategy, trajectory_dir
            )
        else:
            traj_path = None
    else:
        traj_path = None

    t_eval = np.arange(t_skip, tf + DT, DT)
    if len(t_eval) < 10:
        return {'is_breathing': False, 'trajectory_path': traj_path}

    pos_all, vel_all = evaluate(sol, len(xi0), t_eval)
    n_agents = len(xi0)

    r = np.mean(np.linalg.norm(pos_all, axis=2), axis=1)
    r_avg = np.mean(r)
    if r_avg < 0.1:
        return {'is_breathing': False, 'r_avg': r_avg}
    r_min, r_max = np.min(r), np.max(r)
    amp = (r_max - r_min) / r_avg * 100

    min_dist_end = min(np.linalg.norm(pos_all[-1][i] - pos_all[-1][j])
                       for i in range(n_agents) for j in range(i+1, n_agents))

    prominence = max(0.001, 0.005 * r_avg)
    peaks, _ = find_peaks(r, prominence=prominence, distance=10)
    if len(peaks) < 3:
        if amp < 0.3:
            return {'is_breathing': False, 'type': 'equilibrium',
                    'r_avg': r_avg, 'amp': amp, 'min_dist_end': min_dist_end}
        return {'is_breathing': False, 'type': 'no_oscillation',
                'r_avg': r_avg, 'amp': amp, 'min_dist_end': min_dist_end}

    periods = np.diff(t_eval[peaks])
    T = np.mean(periods)
    T_std = np.std(periods)
    T_cv = T_std / T if T > 0 else 999

    omega_eff = 2 * np.pi / T
    ratio = omega_eff / np.sqrt(k2) if k2 > 0 else 0

    is_breathing = (T_cv < 0.05 and amp > 0.5 and min_dist_end > d + 0.1)

    return {
        'is_breathing': is_breathing,
        'type': 'breathing' if is_breathing else 'other',
        'T': T, 'T_std': T_std, 'T_cv': T_cv,
        'omega_eff': omega_eff, 'ratio': ratio,
        'r_avg': r_avg, 'r_min': r_min, 'r_max': r_max,
        'amp': amp,
        'min_dist_end': min_dist_end,
        'trajectory_path': traj_path,
    }


def load_results(N, output_dir):
    """Load existing results for a given N if they exist."""
    output_path = os.path.join(output_dir, f'breathing_N{N}.json')
    if os.path.exists(output_path):
        with open(output_path, 'r') as f:
            data = json.load(f)
        print(f"  [SKIP] N={N}: found {len(data)} existing results, skipping")
        return data
    return None


def save_trajectory(pos_all, vel_all, t_eval, N, k1, k2, d, mu, strategy, output_dir):
    """Save trajectory data to an .npz file with step TRAJECTORY_DT."""
    # Sanitise strategy for use as filename
    safe = strategy.replace('.', '_').replace('=', '_')
    filename = f'traj_N{N}_{safe}.npz'
    out_path = os.path.join(output_dir, filename)
    os.makedirs(output_dir, exist_ok=True)
    np.savez(out_path,
             pos_all=pos_all, vel_all=vel_all, t_eval=t_eval,
             d=d, mu=mu, k1=k1, k2=k2, N=N,
             strategy=strategy,
             description=f'Trajectory for {strategy} (N={N})')
    return out_path


def save_results(N, results, output_dir):
    """Save results for a given N atomically."""
    output_path = os.path.join(output_dir, f'breathing_N{N}.json')
    tmp_path = output_path + '.tmp'
    with open(tmp_path, 'w') as f:
        json.dump(results, f, indent=2, default=str)
    os.replace(tmp_path, output_path)


def search_N(N, output_dir, trajectory_dir=None):
    """Search for breathing limit cycles for a specific N."""
    print(f"\n{'='*60}")
    print(f"N={N}")
    print(f"{'='*60}")

    # Check for existing results
    existing = load_results(N, output_dir)
    if existing is not None:
        return existing

    results = []
    seen = set()
    output_path = os.path.join(output_dir, f'breathing_N{N}.json')

    def add_result(res, strategy):
        if res is None or 'error' in res or res.get('r_avg', 0) < 0.1:
            return
        key = (round(res.get('r_avg', 0), 2), round(res.get('amp', 0), 2))
        if key in seen:
            return
        seen.add(key)
        res['strategy'] = strategy
        res['N'] = N
        results.append(res)
        # Save incrementally after each result
        save_results(N, results, output_dir)
        status = 'BREATHING' if res.get('is_breathing') else res.get('type', '?')
        print(f"  [{status:>12}] {strategy} → "
              f"T={res.get('T', 0):.4f} ratio={res.get('ratio', 0):.4f} "
              f"amp={res.get('amp', 0):.2f}% min_d={res.get('min_dist_end', 0):.3f}")

    # Strategy 1: Regular polygon with perturbations
    for R0 in [4.0, 5.0, 6.0, 7.0]:
        for pert_pos in [0.0, 0.5, 1.0, 2.0]:
            for pert_vel in [0.0, 0.5]:
                seed = int(R0*100) + int(pert_pos*100) + int(pert_vel*100)
                xi0, vt0 = make_regular_polygon(N, R0, pert_pos, pert_vel, seed=seed)
                strategy = f'poly_R{R0}_pp{pert_pos}_pv{pert_vel}'
                res = analyze_trajectory(K1, K2, D, MU, xi0, vt0,
                                         strategy=strategy,
                                         trajectory_dir=trajectory_dir)
                add_result(res, strategy)

    # Strategy 2: Chaotic configurations
    for R0 in [5.0, 6.0, 7.0]:
        for seed in range(5):
            xi0, vt0 = make_chaotic_config(N, R0, seed=seed)
            strategy = f'chaotic_R{R0}_s{seed}'
            res = analyze_trajectory(K1, K2, D, MU, xi0, vt0,
                                     strategy=strategy,
                                     trajectory_dir=trajectory_dir)
            add_result(res, strategy)

    # Final save for this N
    save_results(N, results, output_dir)

    breathing = [r for r in results if r.get('is_breathing')]
    if breathing:
        print(f"\n  >>> N={N}: Found {len(breathing)} breathing limit cycle(s)")
        for b in breathing:
            print(f"      T={b['T']:.4f}s ratio={b['ratio']:.4f} amp={b['amp']:.2f}% "
                  f"r_avg={b['r_avg']:.4f} min_d={b['min_dist_end']:.3f} "
                  f"strategy={b['strategy']}")
    else:
        print(f"\n  >>> N={N}: No breathing limit cycles found")

    return results


def main():
    if len(sys.argv) > 1:
        N_list = [int(x) for x in sys.argv[1:]]
    else:
        N_list = [3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]

    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, '..', 'data', 'breathing_search')
    trajectory_dir = os.path.join(script_dir, '..', 'data', 'trajectories')
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(trajectory_dir, exist_ok=True)

    for N in N_list:
        search_N(N, output_dir, trajectory_dir=trajectory_dir)

    print("\nDone.")


if __name__ == '__main__':
    main()