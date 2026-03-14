# Matlab Wind and Wave Energy Simulation

Hybrid wind–wave generator model built in MATLAB/Simulink, with a Simscape Multibody animation for the wave-side mechanics. This repo keeps only the scripts and documentation; generated models, outputs, and CAD binaries are excluded.

## What’s Inside
- `build_hybrid_model_v4.m`  
  Builds and runs the **hybrid electrical model** (wind + wave), generates figures and CSVs.
- `build_multibody_fixed.m`  
  Builds the **multibody wave mechanism** and wind rotor visual (blade + shaft).
- `run_multibody_animation.m`  
  Helper runner for HT/LT animation without video export.
- `run_full_system_multibody.m`  
  Runs multibody + electrical coupling (if you want combined traces).
- `README_HowToRun.txt`  
  Legacy quick notes.

## Requirements
- MATLAB R2020b+ (Simulink, Simscape, Simscape Multibody)
- Simscape Multibody required for animation (`build_multibody_fixed.m`)

## Quick Start

### 1) Hybrid electrical model (figures + CSV)
```matlab
TIDE_MODE = 'high';  % or 'low'
run('build_hybrid_model_v4.m')
```
Outputs:
- `outputs/figures/Fig4_*_HT.png`
- `outputs/csv/simulation_results_HT.csv`

Repeat with `TIDE_MODE = 'low'` for LT.

### 2) Multibody animation (wave mechanism + wind rotor visual)
```matlab
TIDE_MODE = 'high';     % or 'low'
WIND_SPIN = true;       % set false to keep wind rotor stationary
WIND_SPIN_SCALE = 0.2;  % slow down rotor for visibility (1.0 = real speed)
run('run_multibody_animation.m')
```

## Common Flags (Multibody)
Set these **before** running `build_multibody_fixed.m` or `run_multibody_animation.m`:

- `TIDE_MODE = 'high' | 'low'`  
- `WAVE_DRIVE_MODE = 'motion' | 'force'`  
- `USE_RACK_PINION_CONSTRAINT = true | false`  
- `WIND_SPIN = true | false`  
- `WIND_SPIN_SCALE = 0.0–1.0`  
- `WIND_AXIS = 'X' | 'Y' | 'Z'` (spin axis selector if needed)

## Notes
- CAD/geometry files are not tracked; place your STL files locally and update paths in `build_multibody_fixed.m`.
- Generated `.slx` and output plots/CSVs are ignored; re‑run scripts to regenerate.

