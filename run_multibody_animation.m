%% RUN_MULTIBODY_ANIMATION.M
% Helper script to run Simscape Multibody animation + video export
% Requires MATLAB Desktop (Mechanics Explorer).

cd('c:/Users/User/Vawt_simulink');

% High tide
TIDE_MODE = 'high';
MODEL_SUFFIX = 'HT';
DO_ANIMATE = true;
DO_VIDEO = false; % disable auto video recording to avoid MATLAB crashes
WIND_SPIN = false; % keep wind turbine stationary
WIND_SPIN_SCALE = 0.2; % slow down for stable/visible animation
USE_RACK_PINION_CONSTRAINT = true;
WAVE_DRIVE_MODE = 'motion';
try
    evalc("run('build_multibody_fixed.m');");
catch
    USE_RACK_PINION_CONSTRAINT = false;
    WAVE_DRIVE_MODE = 'motion';
    evalc("run('build_multibody_fixed.m');");
end

% Low tide
TIDE_MODE = 'low';
MODEL_SUFFIX = 'LT';
DO_ANIMATE = true;
DO_VIDEO = false; % disable auto video recording to avoid MATLAB crashes
WIND_SPIN = false; % keep wind turbine stationary
WIND_SPIN_SCALE = 0.2; % slow down for stable/visible animation
USE_RACK_PINION_CONSTRAINT = true;
WAVE_DRIVE_MODE = 'motion';
try
    evalc("run('build_multibody_fixed.m');");
catch
    USE_RACK_PINION_CONSTRAINT = false;
    WAVE_DRIVE_MODE = 'motion';
    evalc("run('build_multibody_fixed.m');");
end
