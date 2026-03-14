HOW TO RUN (HYBRID + MULTIBODY)
===============================
Hybrid Wind-Wave Generator
Bracero | Ligan | Escobido — USTP BSEE 2025-2026

REQUIREMENTS
------------
- MATLAB R2020b or newer
- Simulink
- Simscape (for multibody)

OUTPUT FOLDERS
--------------
Figures:  outputs\figures\
CSVs:     outputs\csv\
Screens:  outputs\screenshots\

HYBRID FIGURES + CSV (v4)
-------------------------
Generates Fig 4.1–4.4 and simulation_results_*.csv.

Run both tides:
  >> cd('C:\Users\User\Vawt_simulink');
  >> USE_MB_INPUTS = false;
  >> TIDE_MODE = 'high'; run('build_hybrid_model_v4.m');
  >> TIDE_MODE = 'low';  run('build_hybrid_model_v4.m');

Notes:
- Duration is controlled in build_hybrid_model_v4.m (t_sim).
- Figures and CSVs are written automatically.

MULTIBODY (WIND STL ONLY)
-------------------------
Uses ONLY the wind turbine STL. Buoy, flywheel, and frame use simple inertia blocks.

Live animation + video (requires MATLAB Desktop):
  >> cd('C:\Users\User\Vawt_simulink');
  >> run('run_multibody_animation.m');

Build model only (no animation/video):
  >> cd('C:\Users\User\Vawt_simulink');
  >> TIDE_MODE = 'high'; MODEL_SUFFIX = 'HT';
  >> DO_ANIMATE = false; DO_VIDEO = false;
  >> run('build_multibody_fixed.m');

Then open the generated model:
  buoy_multibody_fixed_HT.slx

TROUBLESHOOTING
---------------
- If animation is skipped, MATLAB Desktop is not available.
- If Simscape is missing, multibody will not run.
