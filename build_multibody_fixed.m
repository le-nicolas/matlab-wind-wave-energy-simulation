%% =========================================================================
%  BUILD_MULTIBODY_FIXED.M
%  Buoy-Rack-Pinion-Flywheel Multibody - Simscape Multibody (FIXED)
%
%  Authors : Bracero, James J. | Ligan, Carlos Miguel B. | Escobido, Arjie P.
%  School  : USTP BSEE 2025-2026
%
%  FIXES from build_multibody_default.m:
%   1) Wave input supports motion or force actuation
%      - Default = motion (Hs/2 metres) for reliable kinematic drive
%      - Force mode uses InputForce (fallback to InputTorque if needed)
%   2) Transform sensor output uses RConn(2) for omega
%      - RConn(1) is follower frame, RConn(2) is physical signal output
%   3) SL2PS port resolution: use RConn(1) for PS output port
%   4) Flywheel inertia: [Ifly,Ifly,Ifly] -> [Ifly/2, Ifly/2, Ifly]
%      - Disc/ring: Ixx = Iyy = Izz/2 (perpendicular axis theorem)
%   5) Removed spring/damping from prismatic joint
%      - Actuated joint; avoid misleading parameters
%   6) Added Solver Configuration block for Simscape network
%   7) Rack/pinion transforms added (axis/offset tuning hook)
% =========================================================================

% Clear only local variables but keep run-time flags passed in from caller scripts
clearvars -except TIDE_MODE MODEL_SUFFIX DO_ANIMATE DO_VIDEO DO_TRACE WAVE_DRIVE_MODE USE_RACK_PINION_CONSTRAINT WIND_SPIN WIND_SPIN_SCALE WIND_OMEGA_MAX WIND_AXIS used_fallback retried; clc;

%% -------- USER SETTINGS --------------------------------------------------
% Tide selection controls wave height/period constants
if ~exist('TIDE_MODE','var') || isempty(TIDE_MODE)
    TIDE_MODE = 'high';   % 'high' or 'low'
end
% Drive mode for buoy heave: 'motion' (position) is more numerically stable than 'force'
if ~exist('WAVE_DRIVE_MODE','var') || isempty(WAVE_DRIVE_MODE)
    WAVE_DRIVE_MODE = 'motion';   % 'motion' or 'force'
end
% Use Simscape rack-pinion constraint when possible; fallback to kinematic angle if it fails
if ~exist('USE_RACK_PINION_CONSTRAINT','var') || isempty(USE_RACK_PINION_CONSTRAINT)
    USE_RACK_PINION_CONSTRAINT = true;
end
% Kinematic fallback requires motion input
% Kinematic fallback requires motion actuation (force actuation has no effect)
if ~USE_RACK_PINION_CONSTRAINT
    WAVE_DRIVE_MODE = 'motion';
end

% Assign wave climate for each tide condition
if strcmp(TIDE_MODE,'high')
    Hs = 0.841;  T_w = 7.396;  suffix = 'HT';
else
    Hs = 0.657;  T_w = 7.274;  suffix = 'LT';
end

t_sim = 60;
step  = 0.1;

%% -------- ANIMATION / VIDEO ---------------------------------------------
% Animation uses Mechanics Explorer; video export can crash MATLAB on some systems
if ~exist('DO_ANIMATE','var') || isempty(DO_ANIMATE)
    DO_ANIMATE = true;
end
if ~exist('DO_VIDEO','var') || isempty(DO_VIDEO)
    DO_VIDEO = false;
end
% Wind joint axis selector: choose which STL axis should spin about the shaft
if ~exist('WIND_AXIS','var') || isempty(WIND_AXIS)
    WIND_AXIS = 'Z'; % valid: 'X', 'Y', 'Z'
end
% Wind spin scale: reduce for calmer animation (1.0 = real speed)
if ~exist('WIND_SPIN_SCALE','var') || isempty(WIND_SPIN_SCALE)
    WIND_SPIN_SCALE = 1.0;
end
% Safety clamp for wind omega (rad/s) to prevent runaway visuals
if ~exist('WIND_OMEGA_MAX','var') || isempty(WIND_OMEGA_MAX)
    WIND_OMEGA_MAX = 30; % ~286 RPM
end
% Allow freezing the wind turbine rotation for debugging/visual checks
if ~exist('WIND_SPIN','var') || isempty(WIND_SPIN)
    WIND_SPIN = true;
end
VIDEO_FPS    = 30;
VIDEO_FORMAT = 'mpeg-4';

%% -------- WIND + ELECTRICAL PARAMETERS ----------------------------------
% VAWT (Darrieus) parameters used for wind torque and electrical conversion
rho = 1.225;  Cp = 0.30;  TSR = 1.0;
H   = 0.80;   D  = 0.60;  R   = D/2;  A = H*D;

% PMSG constants (voltage per RPM)
k_wind = 24/300;   % V/RPM (wind PMSG)
k_wave = 0.125;    % V/RPM (Quietnight: 1V per 8 RPM)

% MPPT clamp voltage (per source)
V_mppt = 14.4;

%% -------- PROTOTYPE PARAMETERS -------------------------------------------

% Buoy - 10L cylindrical gallon container + ballast (thesis Section 3.7.2)
buoy_r     = 0.127;
buoy_vol_L = 10;
buoy_h     = (buoy_vol_L/1000) / (pi * buoy_r^2);   % = 0.1974 m
buoy_mass  = 3.5;                                     % kg (mid estimate)
buoy_visual_scale = 2.0;                              % visual only (no physics change)
SHOW_BUOY_VISUAL = true;                              % true = show buoy visual

% Wave force (hydrostatic approximation for force-driven mode)
rho_w  = 1025;
g      = 9.81;
A_wp   = pi * buoy_r^2;
F_amp  = 0.5 * rho_w * g * Hs * A_wp;
H_amp  = Hs / 2;  % heave amplitude [m]

% Flywheel - recycled bicycle rim (thesis Section 3.7.3)
fly_mass = 1.5;    % kg
fly_r    = 0.30;   % m
fly_h    = 0.03;   % m

% Gear train - rack-and-pinion 3-stage (thesis handwritten calc)
pitch       = 0.0127;   % 1/2 inch bicycle chain pitch [m]
T_pinion    = 28;
T_36 = 36;  T_18 = 18;
T_20 = 20;  T_9  = 9;
ratio_s2    = T_36 / T_18;          % 2.0
ratio_s3    = T_20 / T_9;           % 2.222
ratio_total = ratio_s2 * ratio_s3;  % 4.444

% Effective pinion radius for RackPinion constraint block
% (converts rack linear speed -> generator shaft angular speed)
r_pinion = (T_pinion * pitch) / (2*pi);
r_eff    = r_pinion / ratio_total;  % 0.012734 m
% Rack/pinion frame offset to satisfy contact geometry (tune if needed)
rack_offset = [0 0 r_eff];
pinion_offset = [0 0 0];

% Load model (simple viscous damping on generator shaft)
B_load = 0.02;                       % N.m.s viscous load
J_fly  = 0.5 * fly_mass * fly_r^2;  % kg.m^2 (spin axis)

% Wind rotor inertia (lumped VAWT)
wind_mass = 5.0;                   % kg (default lumped mass)
wind_r    = R;                      % use VAWT radius
J_wind    = 0.5 * wind_mass * wind_r^2;  % kg.m^2 (spin axis)

% Wind assembly offset (from STL bounding box center of shaft, in mm)
% Add a small scene offset so the wind turbine doesn't overlap the buoy visually.
wind_scene_offset = [2 0 0]; % meters (set [0 0 0] to keep at world origin)
wind_shaft_center_mm = [1418.5019 3413.5449 1477.8839];
wind_blade_center_mm = [1466.3635 3412.9044 1874.9032];
wind_assembly_offset = wind_scene_offset - wind_shaft_center_mm / 1000; % meters
% Blade offset policy:
% For "rotate in place", translation MUST be zero after the revolute joint.
% If CAD is correct (shaft axis at STL origin), leave translation disabled.
USE_BLADE_CENTER_OFFSET = false;
wind_blade_offset = [0 0 0]; % meters (explicitly zero to prevent orbiting)

% STL geometry paths (visuals)
% Use ONLY wind turbine parts (blade + shaft); all other bodies use simple inertia blocks.
buoy_stl        = '';
flywheel_stl    = '';
frame_stl       = '';
wind_blade_stl  = 'C:\Users\User\Downloads\drive-download-20260313T032915Z-1-001\Wind Turbine Blade.stl';
wind_shaft_stl  = 'C:\Users\User\Downloads\drive-download-20260313T032915Z-1-001\Shaft.stl';

fprintf('=== BUILD_MULTIBODY_FIXED.M ===\n');
fprintf('TIDE_MODE = %s | Hs=%.3fm | T_w=%.3fs\n', TIDE_MODE, Hs, T_w);
fprintf('Wave drive = %s | H_amp = %.4f m | F_amp = %.2f N\n', ...
    WAVE_DRIVE_MODE, H_amp, F_amp);
fprintf('Rack/Pinion constraint = %s\n', mat2str(USE_RACK_PINION_CONSTRAINT));
fprintf('r_eff = %.6f m  (gear ratio = %.4f)\n', r_eff, ratio_total);

% Verify RPM against v4
V_rack_check = Hs / (T_w/2);
RPM_expected = ((V_rack_check/(T_pinion*pitch))*60) * ratio_s2 * ratio_s3;
fprintf('Expected generator RPM = %.2f\n\n', RPM_expected);

%% -------- WIND SPEED TIME SERIES ----------------------------------------
wind_vals = [3.0,3.2,3.0,4.0,3.2,3.7,4.2,3.0,4.2,6.0,6.1, ...
             4.0,3.0,2.9,2.7,3.0,2.7,2.2,4.1,3.0,5.0,3.3, ...
             3.0,3.1,4.0,2.0,3.0,3.1,4.0];
wind_t = linspace(0, t_sim, length(wind_vals))';
wind_vals = wind_vals';

% Wind speed input (timeseries) used to derive omega and torque
wind_speed_input = timeseries(wind_vals, wind_t);

% External wind torque from wind speed (TSR-based)
% T_wind = 0.5*rho*Cp*A*(R/TSR)*V^2
wind_torque_vals = 0.5 * rho * Cp * A * (R/TSR) .* (wind_vals.^2);
wind_torque_input = timeseries(wind_torque_vals, wind_t);

% Direct wind omega input from Eq. 3.10 (rad/s); this avoids runaway inertia
wind_omega_vals = (60/(2*pi*R)) .* wind_vals .* (2*pi/60);
% Apply visual scaling + clamp to keep animation stable
wind_omega_vals = wind_omega_vals * WIND_SPIN_SCALE;
wind_omega_vals = min(max(wind_omega_vals, 0), WIND_OMEGA_MAX);
% Optional: freeze wind rotation (set all omega to zero)
if ~WIND_SPIN
    wind_omega_vals = zeros(size(wind_omega_vals));
end
wind_omega_input = timeseries(wind_omega_vals, wind_t);
% Fallback motion input: integrate omega to angle (for position input modes)
wind_theta_vals = cumtrapz(wind_t, wind_omega_vals);
wind_theta_input = timeseries(wind_theta_vals, wind_t);
wind_motion_var = 'wind_omega_input';

fprintf('V_avg = %.4f m/s | T_wind avg = %.4f N*m\n\n', ...
    mean(wind_vals), mean(wind_torque_vals));

%% -------- BUILD MODEL ----------------------------------------------------
if ~exist('MODEL_SUFFIX','var')
    MODEL_SUFFIX = '';
end

mdl = 'buoy_multibody_fixed';
if ~isempty(MODEL_SUFFIX)
    mdl = sprintf('%s_%s', mdl, MODEL_SUFFIX);
end
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end

% Build the Simulink model from scratch each run
new_system(mdl);
open_system(mdl);

% Load required libraries (Simscape Multibody + Simulink-PS converters)
load_system('sm_lib');
load_system('nesl_utility');

% Layout
x0=30; y0=40; dx=180; dy=140; BW=120; BH=60;

% Simulink layout (wind + electrical)
sx0 = x0; sy0 = y0 + 3*dy; sdx = 150; sdy = 90; SBW = 120; SBH = 55;

% Wind mechanical row (separate shaft)
Yw_mech = y0 + 2*dy + 80;

%% --- Mechanism Configuration ---
add_block('sm_lib/Utilities/Mechanism Configuration', [mdl '/MechConfig'], ...
    'Position', [x0 y0 x0+BW y0+BH]);
% Allow kinematic solution for prescribed motion in closed loops
try
    set_param([mdl '/MechConfig'], 'UseKinematicsSolver', 'on');
catch
    try
        set_param([mdl '/MechConfig'], 'UseKinematicSolver', 'on');
    catch
        % Leave default if parameter not available
    end
end

%% --- Solver Configuration (FIX 6) ---
add_block('nesl_utility/Solver Configuration', [mdl '/SolverConfig'], ...
    'Position', [x0 y0+dy+80 x0+BW y0+dy+80+BH]);

%% --- World Frame ---
add_block('sm_lib/Frames and Transforms/World Frame', [mdl '/World'], ...
    'Position', [x0 y0+dy x0+BW y0+dy+BH]);

%% --- Frame Solid (STL, fixed to world) ---
if exist(frame_stl, 'file') == 2
    add_block('sm_lib/Body Elements/File Solid', [mdl '/Frame_Solid'], ...
        'Position', [x0+BW+40 y0+dy x0+BW+40+BW y0+dy+BH]);
    set_param([mdl '/Frame_Solid'], ...
        'ExtGeomFileName', frame_stl, ...
        'UnitType', 'Custom', ...
        'ExtGeomFileUnits', 'm', ...
        'GraphicType', 'FromGeometry', ...
        'InertiaType', 'Custom', ...
        'Mass', '1', ...
        'CenterOfMass', '[0 0 0]', ...
        'MomentsOfInertia', '[1 1 1]', ...
        'ProductsOfInertia', '[0 0 0]');
else
    add_block('sm_lib/Body Elements/Inertia', [mdl '/Frame_Solid'], ...
        'Position', [x0+BW+40 y0+dy x0+BW+40+BW y0+dy+BH]);
    set_param([mdl '/Frame_Solid'], 'InertiaType', 'Custom', ...
        'Mass', '1', 'MomentsOfInertia', '[1 1 1]');
end

%% --- Buoy Solid (STL) ---
Ixx_b = (1/12) * buoy_mass * (3*buoy_r^2 + buoy_h^2);
Izz_b = 0.5 * buoy_mass * buoy_r^2;
if SHOW_BUOY_VISUAL
    if exist(buoy_stl, 'file') == 2
        add_block('sm_lib/Body Elements/File Solid', [mdl '/Buoy_Solid'], ...
            'Position', [x0+3*dx y0+dy x0+3*dx+BW y0+dy+BH]);
        set_param([mdl '/Buoy_Solid'], ...
            'ExtGeomFileName', buoy_stl, ...
            'UnitType', 'Custom', ...
            'ExtGeomFileUnits', 'm', ...
            'GraphicType', 'FromGeometry', ...
            'InertiaType', 'Custom', ...
            'Mass', sprintf('%.6f', buoy_mass), ...
            'CenterOfMass', '[0 0 0]', ...
            'MomentsOfInertia', sprintf('[%.6f %.6f %.6f]', Ixx_b, Ixx_b, Izz_b), ...
            'ProductsOfInertia', '[0 0 0]');
    else
        add_block('sm_lib/Body Elements/Cylindrical Solid', [mdl '/Buoy_Solid'], ...
            'Position', [x0+3*dx y0+dy x0+3*dx+BW y0+dy+BH]);
        set_param([mdl '/Buoy_Solid'], ...
            'CylinderRadius', sprintf('%.6f', buoy_r * buoy_visual_scale), ...
            'CylinderLength', sprintf('%.6f', buoy_h * buoy_visual_scale), ...
            'InertiaType', 'Custom', ...
            'Mass', sprintf('%.6f', buoy_mass), ...
            'CenterOfMass', '[0 0 0]', ...
            'MomentsOfInertia', sprintf('[%.6f %.6f %.6f]', Ixx_b, Ixx_b, Izz_b), ...
            'ProductsOfInertia', '[0 0 0]', ...
            'GraphicType', 'FromGeometry');
    end
else
    % Inertia-only buoy: keep physics, hide visual geometry
    add_block('sm_lib/Body Elements/Inertia', [mdl '/Buoy_Solid'], ...
        'Position', [x0+3*dx y0+dy x0+3*dx+BW y0+dy+BH]);
    set_param([mdl '/Buoy_Solid'], 'InertiaType', 'Custom', ...
        'Mass', sprintf('%.6f', buoy_mass), ...
        'MomentsOfInertia', sprintf('[%.6f %.6f %.6f]', Ixx_b, Ixx_b, Izz_b));
end

%% --- Flywheel Solid (STL) ---
% FIX 4: Ixx=Iyy = Ifly/2 (perpendicular axis theorem for disc/ring)
%         Izz = Ifly (spin axis)
Ifly_zz = J_fly;           % spin axis
Ifly_xx = Ifly_zz / 2;     % perpendicular axes
if exist(flywheel_stl, 'file') == 2
    add_block('sm_lib/Body Elements/File Solid', [mdl '/Flywheel_Solid'], ...
        'Position', [x0+6*dx y0+dy x0+6*dx+BW y0+dy+BH]);
    set_param([mdl '/Flywheel_Solid'], ...
        'ExtGeomFileName', flywheel_stl, ...
        'UnitType', 'Custom', ...
        'ExtGeomFileUnits', 'm', ...
        'GraphicType', 'FromGeometry', ...
        'InertiaType', 'Custom', ...
        'Mass', sprintf('%.6f', fly_mass), ...
        'CenterOfMass', '[0 0 0]', ...
        'MomentsOfInertia', sprintf('[%.6f %.6f %.6f]', Ifly_xx, Ifly_xx, Ifly_zz), ...
        'ProductsOfInertia', '[0 0 0]');
else
    add_block('sm_lib/Body Elements/Inertia', [mdl '/Flywheel_Solid'], ...
        'Position', [x0+6*dx y0+dy x0+6*dx+BW y0+dy+BH]);
    set_param([mdl '/Flywheel_Solid'], 'InertiaType', 'Custom', ...
        'Mass', sprintf('%.6f', fly_mass), ...
        'MomentsOfInertia', sprintf('[%.6f %.6f %.6f]', Ifly_xx, Ifly_xx, Ifly_zz));
end

%% --- Prismatic Joint (buoy heave) ---
% FIX 5: No spring/damping - keep actuation clean
% This joint is the vertical heave of the buoy; we either drive position (motion)
% or apply force depending on WAVE_DRIVE_MODE.
add_block('sm_lib/Joints/Prismatic Joint', [mdl '/Heave_Prismatic'], ...
    'Position', [x0+2*dx y0+dy x0+2*dx+BW y0+dy+BH]);
dp_pris = get_param([mdl '/Heave_Prismatic'], 'DialogParameters');
% Configure actuation depending on drive mode and MATLAB version support
if strcmpi(WAVE_DRIVE_MODE, 'motion')
    if isfield(dp_pris, 'MotionActuationMode')
        set_param([mdl '/Heave_Prismatic'], 'MotionActuationMode', 'InputMotion');
    end
    if isfield(dp_pris, 'ForceActuationMode')
        set_param([mdl '/Heave_Prismatic'], 'ForceActuationMode', 'ComputedForce');
    elseif isfield(dp_pris, 'TorqueActuationMode')
        set_param([mdl '/Heave_Prismatic'], 'TorqueActuationMode', 'ComputedTorque');
    end
else
    if isfield(dp_pris, 'MotionActuationMode')
        set_param([mdl '/Heave_Prismatic'], 'MotionActuationMode', 'ComputedMotion');
    end
    if isfield(dp_pris, 'ForceActuationMode')
        % Correct parameter for prismatic joints (linear actuation)
        set_param([mdl '/Heave_Prismatic'], 'ForceActuationMode', 'InputForce');
    elseif isfield(dp_pris, 'TorqueActuationMode')
        % Fallback for older versions that reuse TorqueActuationMode on prismatic joints
        set_param([mdl '/Heave_Prismatic'], 'TorqueActuationMode', 'InputTorque');
    end
end

%% --- Revolute Joint (generator shaft) ---
% If rack/pinion constraint is active, revolute is driven by the constraint.
% Otherwise we drive the angle directly from rack displacement (kinematic fallback).
add_block('sm_lib/Joints/Revolute Joint', [mdl '/Gen_Revolute'], ...
    'Position', [x0+5*dx y0+dy x0+5*dx+BW y0+dy+BH]);
% Actuation depends on rack/pinion coupling mode
dp_rev = get_param([mdl '/Gen_Revolute'], 'DialogParameters');
if USE_RACK_PINION_CONSTRAINT
    % Let rack/pinion constraint drive the revolute motion
    if isfield(dp_rev, 'TorqueActuationMode')
        set_param([mdl '/Gen_Revolute'], 'TorqueActuationMode', 'NoTorque');
    end
else
    % Kinematic drive of pinion angle
    if isfield(dp_rev, 'MotionActuationMode')
        set_param([mdl '/Gen_Revolute'], 'MotionActuationMode', 'InputMotion');
    end
    if isfield(dp_rev, 'TorqueActuationMode')
        set_param([mdl '/Gen_Revolute'], 'TorqueActuationMode', 'ComputedTorque');
    end
end

%% --- Rack/Pinion Coupling (constraint or kinematic fallback) ---
% Constraint path uses physical rack-pinion contact.
% Fallback path converts rack displacement to pinion angle (stable for animation).
if USE_RACK_PINION_CONSTRAINT
    add_block('sm_lib/Gears and Couplings/Gears/Rack and Pinion Constraint', ...
        [mdl '/RackPinion'], ...
        'Position', [x0+4*dx y0+dy x0+4*dx+BW y0+dy+BH]);
    set_param([mdl '/RackPinion'], 'PinionRadius', sprintf('%.6f', r_eff));

    % Rack alignment (Z -> X) so rack motion matches constraint axis
    add_block('sm_lib/Frames and Transforms/Rigid Transform', [mdl '/Rack_Xform'], ...
        'Position', [x0+3*dx y0+dy x0+3*dx+BW y0+dy+BH]);
    set_param([mdl '/Rack_Xform'], 'RotationMethod','StandardAxis', ...
        'RotationStandardAxis','+Y', ...
        'RotationAngle','-90', ...
        'TranslationMethod','Cartesian', ...
        'TranslationCartesianOffset', sprintf('[%.6f %.6f %.6f]', rack_offset));

    % Pinion offset (tuning hook for alignment)
    add_block('sm_lib/Frames and Transforms/Rigid Transform', [mdl '/Pinion_Xform'], ...
        'Position', [x0+5*dx y0+dy x0+5*dx+BW y0+dy+BH]);
    set_param([mdl '/Pinion_Xform'], 'RotationMethod','None', ...
        'TranslationMethod','Cartesian', ...
        'TranslationCartesianOffset', sprintf('[%.6f %.6f %.6f]', pinion_offset));
else
    % Kinematic fallback: angle = rack_displacement / r_eff
    add_block('simulink/Math Operations/Gain', [mdl '/Rack2Angle'], ...
        'Position', [x0+4*dx y0+2*dy x0+4*dx+BW y0+2*dy+BH], ...
        'Gain', sprintf('%.6f', 1/r_eff));
    add_block('nesl_utility/Simulink-PS Converter', [mdl '/SL2PS_PinionAngle'], ...
        'Position', [x0+5*dx y0+2*dy x0+5*dx+BW y0+2*dy+BH]);
    enable_sl2ps_filtering([mdl '/SL2PS_PinionAngle']);
end

%% --- Transform Sensor (omega only) ---
% Measures flywheel angular velocity for electrical conversion
add_block('sm_lib/Frames and Transforms/Transform Sensor', ...
    [mdl '/OmegaSensor'], ...
    'Position', [x0+7*dx y0+dy x0+7*dx+BW y0+dy+BH]);
set_param([mdl '/OmegaSensor'], 'SenseOmega', 'on');

%% --- Buoy Position Sensor (Z) ---
% Debug sensor to confirm buoy heave motion is actually moving
add_block('sm_lib/Frames and Transforms/Transform Sensor', ...
    [mdl '/Buoy_Pos_Sensor'], ...
    'Position', [x0+4*dx y0+dy-90 x0+4*dx+BW y0+dy-90+BH]);
set_param([mdl '/Buoy_Pos_Sensor'], 'SenseZ', 'on');

add_block('nesl_utility/PS-Simulink Converter', [mdl '/PS2SL_BuoyZ'], ...
    'Position', [x0+5*dx y0+dy-90 x0+5*dx+BW y0+dy-90+BH]);

add_block('simulink/Sinks/To Workspace', [mdl '/Log_BuoyZ'], ...
    'Position', [x0+6*dx y0+dy-90 x0+6*dx+BW y0+dy-90+BH], ...
    'VariableName','ts_buoy_z', ...
    'SaveFormat','Timeseries');

%% --- Wind Shaft (Revolute + Inertia + Omega Sensor) ---
% Align wind STL assembly so the shaft center sits at the joint origin.
% This prevents the turbine from "orbiting" like a planet around the shaft.
add_block('sm_lib/Frames and Transforms/Rigid Transform', [mdl '/Wind_Asm_Xform'], ...
    'Position', [x0+dx Yw_mech-140 x0+dx+BW Yw_mech-140+BH]);
set_param([mdl '/Wind_Asm_Xform'], ...
    'RotationMethod','None', ...
    'TranslationMethod','Cartesian', ...
    'TranslationCartesianOffset', sprintf('[%.6f %.6f %.6f]', wind_assembly_offset));

% Wind joint axis alignment (rotate the joint axis without moving geometry)
add_block('sm_lib/Frames and Transforms/Rigid Transform', [mdl '/Wind_Joint_Xform'], ...
    'Position', [x0+dx Yw_mech-70 x0+dx+BW Yw_mech-70+BH]);
switch upper(WIND_AXIS)
    case 'X'
        % Rotate so joint Z axis aligns with STL X axis
        set_param([mdl '/Wind_Joint_Xform'], 'RotationMethod','StandardAxis', ...
            'RotationStandardAxis','+Y', 'RotationAngle','-90', ...
            'TranslationMethod','None');
    case 'Y'
        % Rotate so joint Z axis aligns with STL Y axis
        set_param([mdl '/Wind_Joint_Xform'], 'RotationMethod','StandardAxis', ...
            'RotationStandardAxis','+X', 'RotationAngle','90', ...
            'TranslationMethod','None');
    otherwise
        % 'Z' -> no rotation
        set_param([mdl '/Wind_Joint_Xform'], 'RotationMethod','None', ...
            'TranslationMethod','None');
end

add_block('sm_lib/Joints/Revolute Joint', [mdl '/Wind_Revolute'], ...
    'Position', [x0+2*dx Yw_mech x0+2*dx+BW Yw_mech+BH]);
dp_wind = get_param([mdl '/Wind_Revolute'], 'DialogParameters');
if isfield(dp_wind, 'MotionActuationMode')
    set_param([mdl '/Wind_Revolute'], 'MotionActuationMode', 'InputMotion');
end
if isfield(dp_wind, 'TorqueActuationMode')
    set_param([mdl '/Wind_Revolute'], 'TorqueActuationMode', 'ComputedTorque');
end
% Feed omega directly as motion input (velocity) so RPM follows the wind time series
wind_motion_is_velocity = false;
try
    set_param([mdl '/Wind_Revolute'], 'MotionInputType', 'Velocity');
    wind_motion_is_velocity = true;
catch
end
if ~wind_motion_is_velocity
    try
        set_param([mdl '/Wind_Revolute'], 'InputType', 'Velocity');
        wind_motion_is_velocity = true;
    catch
    end
end
if ~wind_motion_is_velocity
    % If velocity input is unavailable, fall back to position (angle) input
    try
        set_param([mdl '/Wind_Revolute'], 'MotionInputType', 'Position');
    catch
        try
            set_param([mdl '/Wind_Revolute'], 'InputType', 'Position');
        catch
        end
    end
    wind_motion_var = 'wind_theta_input';
end

Iwind_zz = J_wind;
Iwind_xx = Iwind_zz / 2;
% Fixed shaft (does not rotate) - visual only
if exist(wind_shaft_stl, 'file') == 2
    add_block('sm_lib/Body Elements/File Solid', [mdl '/Wind_Shaft_Solid'], ...
        'Position', [x0+2*dx Yw_mech-90 x0+2*dx+BW Yw_mech-90+BH]);
    set_param([mdl '/Wind_Shaft_Solid'], ...
        'ExtGeomFileName', wind_shaft_stl, ...
        'UnitType', 'Custom', ...
        'ExtGeomFileUnits', 'mm', ... % STL exported in millimeters
        'GraphicType', 'FromGeometry', ...
        'InertiaType', 'Custom', ...
        'Mass', '1', ...
        'CenterOfMass', '[0 0 0]', ...
        'MomentsOfInertia', '[1 1 1]', ...
        'ProductsOfInertia', '[0 0 0]');
else
    add_block('sm_lib/Body Elements/Inertia', [mdl '/Wind_Shaft_Solid'], ...
        'Position', [x0+2*dx Yw_mech-90 x0+2*dx+BW Yw_mech-90+BH]);
    set_param([mdl '/Wind_Shaft_Solid'], 'InertiaType', 'Custom', ...
        'Mass', '1', 'MomentsOfInertia', '[1 1 1]');
end

% Rotating blades (only these spin) - driven by Wind_Revolute
if exist(wind_blade_stl, 'file') == 2
    add_block('sm_lib/Body Elements/File Solid', [mdl '/Wind_Blade_Solid'], ...
        'Position', [x0+3*dx Yw_mech x0+3*dx+BW Yw_mech+BH]);
    set_param([mdl '/Wind_Blade_Solid'], ...
        'ExtGeomFileName', wind_blade_stl, ...
        'UnitType', 'Custom', ...
        'ExtGeomFileUnits', 'mm', ... % STL exported in millimeters
        'GraphicType', 'FromGeometry', ...
        'InertiaType', 'Custom', ...
        'Mass', sprintf('%.6f', wind_mass), ...
        'CenterOfMass', '[0 0 0]', ...
        'MomentsOfInertia', sprintf('[%.6f %.6f %.6f]', Iwind_xx, Iwind_xx, Iwind_zz), ...
        'ProductsOfInertia', '[0 0 0]');
else
    add_block('sm_lib/Body Elements/Inertia', [mdl '/Wind_Blade_Solid'], ...
        'Position', [x0+3*dx Yw_mech x0+3*dx+BW Yw_mech+BH]);
    set_param([mdl '/Wind_Blade_Solid'], 'InertiaType', 'Custom', ...
        'Mass', sprintf('%.6f', wind_mass), ...
        'MomentsOfInertia', sprintf('[%.6f %.6f %.6f]', Iwind_xx, Iwind_xx, Iwind_zz));
end

% Offset blade geometry so its local origin sits on the shaft axis.
% Translation is DISABLED to guarantee pure rotation about the shaft axis.
add_block('sm_lib/Frames and Transforms/Rigid Transform', [mdl '/Wind_Blade_Xform'], ...
    'Position', [x0+3*dx Yw_mech+90 x0+3*dx+BW Yw_mech+90+BH]);
set_param([mdl '/Wind_Blade_Xform'], ...
    'RotationMethod','None', ...
    'TranslationMethod','None');

add_block('sm_lib/Frames and Transforms/Transform Sensor', ...
    [mdl '/Wind_OmegaSensor'], ...
    'Position', [x0+4*dx Yw_mech x0+4*dx+BW Yw_mech+BH]);
set_param([mdl '/Wind_OmegaSensor'], 'SenseOmega', 'on');

add_block('simulink/Sources/From Workspace', [mdl '/Wind_Omega'], ...
    'Position', [x0 Yw_mech x0+BW Yw_mech+BH], ...
    'VariableName', wind_motion_var, ...
    'SampleTime', '0', ...
    'Interpolate', 'on');

add_block('nesl_utility/Simulink-PS Converter', [mdl '/SL2PS_WindOmega'], ...
    'Position', [x0+dx Yw_mech x0+dx+BW Yw_mech+BH]);
enable_sl2ps_filtering([mdl '/SL2PS_WindOmega']);

add_block('nesl_utility/PS-Simulink Converter', [mdl '/PS2SL_WindOmega'], ...
    'Position', [x0+5*dx Yw_mech x0+5*dx+BW Yw_mech+BH]);

%% --- Wind omega magnitude (vector -> scalar) ---
add_block('simulink/Math Operations/Dot Product', [mdl '/Wind_Omega_Dot'], ...
    'Position', [x0+5*dx+BW+20 Yw_mech x0+5*dx+BW+80 Yw_mech+BH]);
add_block('simulink/Math Operations/Math Function', [mdl '/Wind_Omega_Sqrt'], ...
    'Position', [x0+5*dx+BW+110 Yw_mech x0+5*dx+BW+170 Yw_mech+BH], ...
    'Function', 'sqrt');

%% --- Wave Force Input ---
% Wave input (motion in metres or force in N)
if strcmpi(WAVE_DRIVE_MODE, 'motion')
    wave_amp = H_amp;
    wave_units = 'm';
else
    wave_amp = F_amp;
    wave_units = 'N';
end
add_block('simulink/Sources/Sine Wave', [mdl '/WaveSine'], ...
    'Position', [x0+2*dx y0+2*dy x0+2*dx+BW y0+2*dy+BH], ...
    'Amplitude',  sprintf('%.6f', wave_amp), ...
    'Frequency',  sprintf('%.6f', 2*pi/T_w), ...
    'SampleTime', '0');

%% --- Simulink-PS Converter ---
add_block('nesl_utility/Simulink-PS Converter', [mdl '/SL2PS'], ...
    'Position', [x0+2*dx+BW+20 y0+2*dy x0+2*dx+BW+90 y0+2*dy+BH]);
enable_sl2ps_filtering([mdl '/SL2PS']);

%% --- PS-Simulink Converter (omega out) ---
add_block('nesl_utility/PS-Simulink Converter', [mdl '/PS2SL_Omega'], ...
    'Position', [x0+8*dx y0+dy x0+8*dx+BW y0+dy+BH]);

%% --- Omega magnitude (vector -> scalar) ---
add_block('simulink/Math Operations/Dot Product', [mdl '/Omega_Dot'], ...
    'Position', [x0+8*dx+BW+20 y0+dy x0+8*dx+BW+80 y0+dy+BH]);
add_block('simulink/Math Operations/Math Function', [mdl '/Omega_Sqrt'], ...
    'Position', [x0+8*dx+BW+110 y0+dy x0+8*dx+BW+170 y0+dy+BH], ...
    'Function', 'sqrt');

%% --- RPM Gain ---
add_block('simulink/Math Operations/Gain', [mdl '/Rad2RPM'], ...
    'Position', [x0+9*dx y0+dy x0+9*dx+BW y0+dy+BH], ...
    'Gain', sprintf('%.6f', 60/(2*pi)));

%% --- Torque Estimate: T = J*alpha + B*omega ---
add_block('simulink/Continuous/Derivative', [mdl '/dOmega'], ...
    'Position', [x0+9*dx y0+dy+80 x0+9*dx+BW y0+dy+80+BH]);
add_block('simulink/Math Operations/Gain', [mdl '/Jalpha'], ...
    'Position', [x0+10*dx y0+dy+80 x0+10*dx+BW y0+dy+80+BH], ...
    'Gain', sprintf('%.6f', J_fly));
add_block('simulink/Math Operations/Gain', [mdl '/Bomega'], ...
    'Position', [x0+10*dx y0+dy x0+10*dx+BW y0+dy+BH], ...
    'Gain', sprintf('%.6f', B_load));
add_block('simulink/Math Operations/Add', [mdl '/TorqueSum'], ...
    'Position', [x0+11*dx y0+dy+40 x0+11*dx+BW y0+dy+40+BH], ...
    'Inputs', '++');

%% --- WIND ELECTRICAL (SEPARATE SHAFT) ---
% Wind torque input (external)
add_block('simulink/Sources/From Workspace', [mdl '/Wind_Torque'], ...
    'Position', [sx0 sy0+sdy sx0+SBW sy0+sdy+SBH], ...
    'VariableName', 'wind_torque_input', ...
    'SampleTime', '0', ...
    'Interpolate', 'on');

% Wind RPM from multibody omega
add_block('simulink/Math Operations/Gain', [mdl '/Wind_RPM'], ...
    'Position', [sx0+2*sdx sy0 sx0+2*sdx+SBW sy0+SBH], ...
    'Gain', sprintf('%.10f', 60/(2*pi)));

% V_AC_wind = k_wind * RPM_wind
add_block('simulink/Math Operations/Gain', [mdl '/Wind_PMSG'], ...
    'Position', [sx0+3*sdx sy0 sx0+3*sdx+SBW sy0+SBH], ...
    'Gain', sprintf('%.10f', k_wind));

% V_DC_wind = V_AC * 0.9 * 0.707
add_block('simulink/Math Operations/Gain', [mdl '/Wind_Rect'], ...
    'Position', [sx0+4*sdx sy0 sx0+4*sdx+SBW sy0+SBH], ...
    'Gain', sprintf('%.10f', 0.9*0.707));

% MPPT clamp (wind)
add_block('simulink/Discontinuities/Saturation', [mdl '/MPPT_Wind'], ...
    'Position', [sx0+5*sdx sy0 sx0+5*sdx+SBW sy0+SBH], ...
    'UpperLimit', sprintf('%.2f', V_mppt), ...
    'LowerLimit', '0');

%% --- WAVE ELECTRICAL (FROM MULTIBODY RPM) ---
% Quietnight spec is DC already (no rectifier gain needed)
add_block('simulink/Math Operations/Gain', [mdl '/Wave_PMSG'], ...
    'Position', [sx0+3*sdx sy0+2*sdy sx0+3*sdx+SBW sy0+2*sdy+SBH], ...
    'Gain', sprintf('%.10f', k_wave));

add_block('simulink/Math Operations/Gain', [mdl '/Wave_Rect'], ...
    'Position', [sx0+4*sdx sy0+2*sdy sx0+4*sdx+SBW sy0+2*sdy+SBH], ...
    'Gain', '1.0');

add_block('simulink/Discontinuities/Saturation', [mdl '/MPPT_Wave'], ...
    'Position', [sx0+5*sdx sy0+2*sdy sx0+5*sdx+SBW sy0+2*sdy+SBH], ...
    'UpperLimit', sprintf('%.2f', V_mppt), ...
    'LowerLimit', '0');

% DC Bus sum (electrical combine)
add_block('simulink/Math Operations/Add', [mdl '/DC_Bus'], ...
    'Position', [sx0+6*sdx sy0+2*sdy sx0+6*sdx+SBW sy0+2*sdy+SBH], ...
    'Inputs', '++');

%% --- Outports ---
add_block('simulink/Sinks/Out1', [mdl '/Out_RPM'], ...
    'Position', [x0+12*dx y0+dy x0+12*dx+BW y0+dy+BH]);
set_param([mdl '/Out_RPM'], 'Port', '1');
add_block('simulink/Sinks/Out1', [mdl '/Out_Torque'], ...
    'Position', [x0+12*dx y0+dy+80 x0+12*dx+BW y0+dy+80+BH]);
set_param([mdl '/Out_Torque'], 'Port', '2');

add_block('simulink/Sinks/Out1', [mdl '/Out_RPM_Wind'], ...
    'Position', [sx0+6*sdx sy0 sx0+6*sdx+SBW sy0+SBH]);
set_param([mdl '/Out_RPM_Wind'], 'Port', '3');

add_block('simulink/Sinks/Out1', [mdl '/Out_Torque_Wind'], ...
    'Position', [sx0+6*sdx sy0+sdy sx0+6*sdx+SBW sy0+sdy+SBH]);
set_param([mdl '/Out_Torque_Wind'], 'Port', '4');

add_block('simulink/Sinks/Out1', [mdl '/Out_VDC_Bus'], ...
    'Position', [sx0+7*sdx sy0+2*sdy sx0+7*sdx+SBW sy0+2*sdy+SBH]);
set_param([mdl '/Out_VDC_Bus'], 'Port', '5');

%% -------- CONNECTIONS ----------------------------------------------------

% Grab port handles once so connections are readable and version-safe
phWorld = get_param([mdl '/World'],           'PortHandles');
phPris  = get_param([mdl '/Heave_Prismatic'], 'PortHandles');
phBuoy  = get_param([mdl '/Buoy_Solid'],      'PortHandles');
phRev   = get_param([mdl '/Gen_Revolute'],    'PortHandles');
phFly   = get_param([mdl '/Flywheel_Solid'],  'PortHandles');
if USE_RACK_PINION_CONSTRAINT
    phRP    = get_param([mdl '/RackPinion'],       'PortHandles');
    phRackX = get_param([mdl '/Rack_Xform'],        'PortHandles');
    phPinX  = get_param([mdl '/Pinion_Xform'],      'PortHandles');
else
    phRack2 = get_param([mdl '/Rack2Angle'],        'PortHandles');
    phSL2PS_Pin = get_param([mdl '/SL2PS_PinionAngle'], 'PortHandles');
end
phTS    = get_param([mdl '/OmegaSensor'],      'PortHandles');
phBuoyTS= get_param([mdl '/Buoy_Pos_Sensor'],   'PortHandles');
phSL2PS = get_param([mdl '/SL2PS'],            'PortHandles');
phPS2SL = get_param([mdl '/PS2SL_Omega'],      'PortHandles');
phPS2SL_BZ = get_param([mdl '/PS2SL_BuoyZ'],   'PortHandles');
phSolver= get_param([mdl '/SolverConfig'],     'PortHandles');
phFrame = get_param([mdl '/Frame_Solid'],      'PortHandles');

phWindRev = get_param([mdl '/Wind_Revolute'],     'PortHandles');
phWindAsm = get_param([mdl '/Wind_Asm_Xform'],    'PortHandles');
phWindJoint = get_param([mdl '/Wind_Joint_Xform'],'PortHandles');
phWindShaft = get_param([mdl '/Wind_Shaft_Solid'],'PortHandles');
phWindBlade = get_param([mdl '/Wind_Blade_Solid'],'PortHandles');
phWindBladeX = get_param([mdl '/Wind_Blade_Xform'],'PortHandles');
phWindTS  = get_param([mdl '/Wind_OmegaSensor'],  'PortHandles');
phSL2PS_W = get_param([mdl '/SL2PS_WindOmega'],  'PortHandles');
phPS2SL_W = get_param([mdl '/PS2SL_WindOmega'],  'PortHandles');

% FIX 3: SL2PS PS output port = RConn(1) (physical signal output)
sl2ps_ps_out = phSL2PS.RConn(1);

% FIX 2: Omega output is RConn(2) when SenseOmega is enabled
% (RConn(1) is follower frame, RConn(2) is physical signal)
ts_omega_out = phTS.RConn(2);

% Buoy position output (SenseZ) is RConn(2)
ts_buoy_out = phBuoyTS.RConn(2);

% PS2SL physical input port (physical -> Simulink)
ps2sl_ps_in = phPS2SL.LConn(1);

% Wind motion + omega ports (wind sensor + actuation)
sl2ps_wind_out = phSL2PS_W.RConn(1);
wind_omega_out = phWindTS.RConn(2);
ps2sl_wind_in  = phPS2SL_W.LConn(1);

%% Physical network connections
% Simscape physical lines connect frames/physical signals (not Simulink signals)

% Solver Config -> World (required to assemble the physical network)
add_line(mdl, phSolver.RConn(1), phWorld.RConn(1), 'autorouting','on');

% World -> Frame solid (fixed visual)
add_line(mdl, phWorld.RConn(1), phFrame.RConn(1), 'autorouting','on');

% Wind shaft (separate mechanical path - not mechanically connected to buoy/flywheel)
add_line(mdl, phWorld.RConn(1),  phWindAsm.LConn(1), 'autorouting','on');
add_line(mdl, phWindAsm.RConn(1), phWindJoint.LConn(1), 'autorouting','on');
add_line(mdl, phWindJoint.RConn(1), phWindRev.LConn(1), 'autorouting','on');
add_line(mdl, phWindRev.RConn(1), phWindBladeX.LConn(1), 'autorouting','on');
add_line(mdl, phWindBladeX.RConn(1), phWindBlade.RConn(1), 'autorouting','on');
add_line(mdl, sl2ps_wind_out,     phWindRev.LConn(2), 'autorouting','on');
% Fixed shaft body (no joint) - purely visual anchor
add_line(mdl, phWindAsm.RConn(1), phWindShaft.RConn(1), 'autorouting','on');

% Wind omega sensor
add_line(mdl, phWindAsm.RConn(1), phWindTS.LConn(1), 'autorouting','on');
add_line(mdl, phWindBlade.RConn(1), phWindTS.RConn(1), 'autorouting','on');
add_line(mdl, wind_omega_out,     ps2sl_wind_in,     'autorouting','on');

% World -> Prismatic (base frame for buoy heave)
add_line(mdl, phWorld.RConn(1), phPris.LConn(1),  'autorouting','on');
% Prismatic follower -> Buoy
add_line(mdl, phPris.RConn(1),  phBuoy.RConn(1),  'autorouting','on');

% World -> Revolute (base frame for generator shaft)
add_line(mdl, phWorld.RConn(1), phRev.LConn(1),   'autorouting','on');
% Revolute follower -> Flywheel
add_line(mdl, phRev.RConn(1),   phFly.RConn(1),   'autorouting','on');

% Buoy position sensor: World -> base, Buoy -> follower
add_line(mdl, phWorld.RConn(1), phBuoyTS.LConn(1), 'autorouting','on');
add_line(mdl, phBuoy.RConn(1),  phBuoyTS.RConn(1), 'autorouting','on');
add_line(mdl, ts_buoy_out,      phPS2SL_BZ.LConn(1), 'autorouting','on');

if USE_RACK_PINION_CONSTRAINT
    % Rack and Pinion: rack side = prismatic follower, pinion side = revolute follower
    add_line(mdl, phPris.RConn(1),  phRackX.LConn(1), 'autorouting','on');
    add_line(mdl, phRackX.RConn(1), phRP.LConn(1),    'autorouting','on');
    add_line(mdl, phRev.RConn(1),   phPinX.LConn(1),  'autorouting','on');
    add_line(mdl, phPinX.RConn(1),  phRP.RConn(1),    'autorouting','on');
else
    % Kinematic coupling: WaveSine (rack displacement) -> pinion angle input
    add_line(mdl, 'WaveSine/1',      'Rack2Angle/1',      'autorouting','on');
    add_line(mdl, 'Rack2Angle/1',    'SL2PS_PinionAngle/1','autorouting','on');
    add_line(mdl, phSL2PS_Pin.RConn(1), phRev.LConn(2),    'autorouting','on');
end

% Wave sine -> SL2PS -> Prismatic motion input port
add_line(mdl, 'WaveSine/1', 'SL2PS/1',            'autorouting','on');
add_line(mdl, sl2ps_ps_out, phPris.LConn(2),       'autorouting','on');

% Transform sensor: World -> base, Flywheel -> follower
add_line(mdl, phWorld.RConn(1), phTS.LConn(1),    'autorouting','on');
add_line(mdl, phFly.RConn(1),   phTS.RConn(1),    'autorouting','on');

% Omega output -> PS2SL
add_line(mdl, ts_omega_out, ps2sl_ps_in,           'autorouting','on');

%% Simulink signal connections
% Standard Simulink lines for logging and electrical calculations

% Omega -> RPM and torque estimate (vector magnitude)
add_line(mdl, 'PS2SL_Omega/1', 'Omega_Dot/1',      'autorouting','on');
add_line(mdl, 'PS2SL_Omega/1', 'Omega_Dot/2',      'autorouting','on');
add_line(mdl, 'Omega_Dot/1',   'Omega_Sqrt/1',     'autorouting','on');
add_line(mdl, 'Omega_Sqrt/1',  'Rad2RPM/1',        'autorouting','on');
add_line(mdl, 'Omega_Sqrt/1',  'Bomega/1',         'autorouting','on');
add_line(mdl, 'Omega_Sqrt/1',  'dOmega/1',         'autorouting','on');
add_line(mdl, 'dOmega/1',      'Jalpha/1',          'autorouting','on');
add_line(mdl, 'Jalpha/1',      'TorqueSum/1',       'autorouting','on');
add_line(mdl, 'Bomega/1',      'TorqueSum/2',       'autorouting','on');

% Wind lane connections (drive revolute with omega and log RPM)
add_line(mdl, 'Wind_Omega/1',      'SL2PS_WindOmega/1', 'autorouting','on');
add_line(mdl, 'PS2SL_WindOmega/1', 'Wind_Omega_Dot/1',  'autorouting','on');
add_line(mdl, 'PS2SL_WindOmega/1', 'Wind_Omega_Dot/2',  'autorouting','on');
add_line(mdl, 'Wind_Omega_Dot/1',  'Wind_Omega_Sqrt/1', 'autorouting','on');
add_line(mdl, 'Wind_Omega_Sqrt/1', 'Wind_RPM/1',        'autorouting','on');

% Buoy position logging
add_line(mdl, 'PS2SL_BuoyZ/1', 'Log_BuoyZ/1', 'autorouting','on');
add_line(mdl, 'Wind_RPM/1',        'Wind_PMSG/1',      'autorouting','on');
add_line(mdl, 'Wind_PMSG/1',       'Wind_Rect/1',      'autorouting','on');
add_line(mdl, 'Wind_Rect/1',       'MPPT_Wind/1',      'autorouting','on');
add_line(mdl, 'MPPT_Wind/1',       'DC_Bus/1',         'autorouting','on');
add_line(mdl, 'Wind_RPM/1',        'Out_RPM_Wind/1',   'autorouting','on');
add_line(mdl, 'Wind_Torque/1',     'Out_Torque_Wind/1','autorouting','on');

% Wave electrical lane
add_line(mdl, 'Rad2RPM/1',    'Wave_PMSG/1',         'autorouting','on');
add_line(mdl, 'Wave_PMSG/1',  'Wave_Rect/1',         'autorouting','on');
add_line(mdl, 'Wave_Rect/1',  'MPPT_Wave/1',         'autorouting','on');
add_line(mdl, 'MPPT_Wave/1',  'DC_Bus/2',            'autorouting','on');
add_line(mdl, 'DC_Bus/1',     'Out_VDC_Bus/1',       'autorouting','on');

% Outputs
add_line(mdl, 'Rad2RPM/1',    'Out_RPM/1',          'autorouting','on');
add_line(mdl, 'TorqueSum/1',  'Out_Torque/1',        'autorouting','on');

%% -------- SIM SETTINGS ---------------------------------------------------
% Use a stiff solver (ode15s) for multibody stability
set_param(mdl, 'StopTime',  sprintf('%.1f', t_sim), ...
               'Solver',    'ode15s', ...
               'MaxStep',   sprintf('%.2f', step));

save_system(mdl, [mdl '.slx']);

fprintf('Model saved: %s.slx\n', mdl);

%% --- EXPORT MODEL SCREENSHOT ---
% Quick layout image for documentation without opening Mechanics Explorer
outdir_img = fullfile(pwd, 'outputs', 'screenshots');
if ~exist(outdir_img, 'dir'), mkdir(outdir_img); end
img_file = fullfile(outdir_img, sprintf('multibody_layout_%s.png', suffix));

try
    set_param(mdl, 'ZoomFactor', 'FitSystem');
    print(['-s' mdl], '-dpng', img_file);
    fprintf('Saved screenshot: %s\n', img_file);
catch ME
    warning('Screenshot export failed: %s', ME.message);
end

%% --- ANIMATION + VIDEO EXPORT ---
% Only runs if MATLAB Desktop is available; DO_VIDEO can be disabled to avoid crashes
if DO_ANIMATE || DO_VIDEO
    if usejava('desktop')
        % Try to ensure Mechanics Explorer opens on update
        try
            set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'on');
        catch
        end
        try
            set_param(mdl, 'SimscapeMultibodyOpenEditorOnUpdate', 'on');
        catch
        end

        disp('Running simulation for animation/video...');
        sim(mdl);

        if DO_VIDEO
            outdir_vid = fullfile(pwd, 'outputs', 'videos');
            if ~exist(outdir_vid, 'dir'), mkdir(outdir_vid); end
            vid_file = fullfile(outdir_vid, sprintf('multibody_%s.mp4', suffix));
            try
                smwritevideo(mdl, vid_file, 'VideoFormat', VIDEO_FORMAT, 'FrameRate', VIDEO_FPS);
                fprintf('Saved video: %s\n', vid_file);
            catch ME
                warning('MP4 export failed: %s', ME.message);
                vid_file = fullfile(outdir_vid, sprintf('multibody_%s.avi', suffix));
                smwritevideo(mdl, vid_file, 'VideoFormat', 'motion jpeg avi', 'FrameRate', VIDEO_FPS);
                fprintf('Saved video (AVI): %s\n', vid_file);
            end
        end
    else
        disp('Skipping animation/video: MATLAB Desktop not available.');
    end
end

try
    fprintf('\nALL 7 BUGS FIXED:\n');
    fprintf('  1. WaveSine amplitude = %.4f %s (%s input)\n', wave_amp, wave_units, WAVE_DRIVE_MODE);
    fprintf('  2. Transform sensor omega port = RConn(2)\n');
    fprintf('  3. SL2PS PS output port = RConn(1)\n');
    fprintf('  4. Flywheel [Ixx Iyy Izz] = [%.4f %.4f %.4f]\n', Ifly_xx, Ifly_xx, Ifly_zz);
    fprintf('  5. Prismatic spring/damping removed (clean actuation)\n');
    fprintf('  6. Solver Configuration added\n');
    fprintf('  7. Rack/pinion transform blocks added (tuning hook)\n');
    fprintf('\nExpected generator RPM: %.2f (%s)\n', RPM_expected, suffix);
catch
    % Avoid batch run failure if output stream is unavailable.
end

%% --- Local helper: enable SL2PS input filtering (robust to version) ---
function enable_sl2ps_filtering(block)
    % R2025b+: set explicit filtering mode
    try
        set_param(block, 'FilteringAndDerivatives', 'filter');
    catch
    end
    try
        set_param(block, 'InputFilterTimeConstant', '0.01');
    catch
    end
    try
        set_param(block, 'SimscapeFilterOrder', '2');
    catch
    end
    % Backward-compat candidates
    candidates = { ...
        {'InputHandling','Filtering'}, ...
        {'InputHandling','Filter'}, ...
        {'Filtering','on'}, ...
        {'UseInputFiltering','on'}, ...
        {'InputFiltering','on'}, ...
        {'FilterTimeConstant','0.01'} ...
    };
    for i = 1:numel(candidates)
        kv = candidates{i};
        try
            set_param(block, kv{1}, kv{2});
        catch
        end
    end
end
