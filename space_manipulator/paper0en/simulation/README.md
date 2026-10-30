# Simulation code for the paper

The results reported in Section VI of the paper are produced by

```bash
/Applications/MATLAB_R2025b.app/bin/matlab -batch "main_ffsm_pdf_tracking_dou2025('full')" -singleCompThread
```

from this directory (MATLAB R2025b; no toolboxes required).

## Files

| File | Purpose |
|---|---|
| `main_ffsm_pdf_tracking_dou2025.m` | Main study: plant, controller (PD / PDF / PDF+PTDO), six cases, metrics, figure export |
| `plot_paper_figures.m` | Exports each paper figure as an independent PDF/PNG pair into `../figures` |
| `dou2025_parameters.m` | Kinematic and dynamic parameters of the 7-DOF manipulator (Dou & Yue, IJRC 2025, Tables 1–2) |
| `ffsm_dou_crba.m` | Spatial composite rigid-body algorithm: full $(6+n)\times(6+n)$ inertia matrix, MDH chain |
| `ffsm_dou_bias.m` | Recursive Newton–Euler bias forces $C(q,\nu)\nu$ at the momentum-consistent base velocity |
| `ffsm_dou_reduced.m` | Equivalent joint-space model: Schur complement $M_e$ and $C_e\dot q$ |
| `ffsm_dou_fk.m` | Forward kinematics (joint origins, end-effector pose) in the base frame |
| `model_selfcheck.m` | Model consistency checks (positive definiteness, momentum, skew-symmetry, energy) |
| `dou2025_results.mat` | Saved results of the last full run |

The figures in `../figures` are overwritten by every full run.

## Older material (superseded, kept for reference)

`main_ffsm_pdf_tracking_yan_params.m`, `run_final_paper_sim.m`,
`plot_ffsm_pdf_tracking_results.m`, `compute_*.m` and `latest_results.mat`
belong to the earlier version of the study, which used a nominal joint-space
model with a different parameter set (Yan's thesis). They are **not** used by
the current paper.
