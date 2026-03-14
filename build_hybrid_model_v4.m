%% =========================================================================
%  BUILD_HYBRID_MODEL_V4.M
%  Hybrid Wind-Wave Generator — Simulink Model (FINAL — Real Parameters)
%
%  Authors : Bracero, James J. | Ligan, Carlos Miguel B. | Escobido, Arjie P.
%  School  : University of Science and Technology of Southern Philippines (USTP)
%  Site    : San Pedro Beach, Opol, Misamis Oriental
%
%  REAL PARAMETERS (updated from actual prototype and product specs):
%
%  WAVE GENERATOR (Quietnight, Shopee):
%    - 200W brushless AC PMSG
%    - Spec: 96 RPM → 12V DC (after rectification)
%    - 1V per 8 RPM → k_wave = 0.125 V_DC/RPM
%
%  GEAR TRAIN (actual fabricated rack-and-pinion, 3-stage):
%    Stage 1: Buoy → sliding rack → 28T pinion (bicycle chain pitch 0.0127m)
%    Stage 2: 36T (same shaft as 28T) → 18T sprocket  (ratio 2:1)
%    Stage 3: 20T → 9T via flywheel rim              (ratio 2.22:1)
%    Total effective ratio varies with wave conditions (not fixed 20:1)
%    HT: RPM_gen = 170.54 | LT: RPM_gen = 135.47
%
%  WIND GENERATOR:
%    - 500W Darrieus VAWT, 5-blade, 12V rated
%    - k_wind = 0.08 V_AC/RPM (rated 12V output at ~300 RPM operation)
%    - RPM = 60 × V / (2π × R)   [thesis eq. 3.10]
%
%  WAVE DATA (thesis Table 3.2):
%    High Tide: Hs=0.841m, T=7.396s
%    Low Tide:  Hs=0.657m, T=7.274s
%    Overall avg (gear design basis): Hs=0.81m, T=7.47s
%
%  HOW TO USE:
%    >> run('build_hybrid_model_v4.m')
%    Change TIDE_MODE = 'low' for Low Tide run
% =========================================================================

% Clear local variables but keep optional inputs passed from multibody runs
clearvars -except TIDE_MODE USE_MB_INPUTS wind_rpm_input wave_rpm_input wind_torque_input tide_list k; clc; close all;
disp('=============================================================');
disp(' HYBRID WIND-WAVE GENERATOR v4 — REAL PARAMETERS');
disp(' USTP BSEE 2025-2026 | Bracero · Ligan · Escobido');
disp('=============================================================');

% Optional: use multibody inputs for wind/wave RPM
if ~exist('USE_MB_INPUTS','var') || isempty(USE_MB_INPUTS)
    USE_MB_INPUTS = false;
end

%% =========================================================================
%  SECTION 1: TIDE MODE
% =========================================================================
% Tide mode selects the wave height/period used everywhere downstream
if ~exist('TIDE_MODE','var') || isempty(TIDE_MODE)
    TIDE_MODE = 'high';   % default
end

% Assign tidal constants and label for plots/filenames
if strcmp(TIDE_MODE,'high')
    Hs    = 0.841;   T_w = 7.396;
    label = 'High Tide (Hs=0.841m, T=7.396s)';
    suffix = 'HT';
else
    Hs    = 0.657;   T_w = 7.274;
    label = 'Low Tide (Hs=0.657m, T=7.274s)';
    suffix = 'LT';
end
fprintf(' Mode: %s\n\n', label);

%% =========================================================================
%  SECTION 2: FIXED DESIGN PARAMETERS
% =========================================================================

% VAWT (thesis measured dimensions, Cp from Ragheb 2019)
rho  = 1.225;   Cp = 0.30;   TSR = 1.0;
H    = 0.80;    D  = 0.60;   R   = D/2;   A = H*D;

% Wind PMSG (500W Darrieus, 12V rated)
k_wind = 0.08;      % V_AC/RPM — rated 12V DC output
% Wind shaft viscous load (to stabilize RPM when using multibody inputs)
B_wind = 0.03;      % N·m·s (tuned for ~111 RPM avg)

% Wave PMSG (Quietnight 200W brushless)
% Real spec: 96 RPM → 12V DC,  1V per 8 RPM
k_wave = 0.125;     % V_DC/RPM — direct DC output (already post-rectification)

% Gear train (actual fabricated: rack-and-pinion 3-stage)
pitch     = 0.0127;   % 1/2 inch standard bicycle chain pitch [m]
T_pinion  = 28;       % Rack pinion teeth
T_36      = 36;       % Stage 2 input (same shaft as 28T pinion)
T_18      = 18;       % Stage 2 output
T_20      = 20;       % Stage 3 input (on flywheel rim)
T_9       = 9;        % Stage 3 output (generator shaft)
C_pinion  = T_pinion * pitch;          % Pinion circumference [m]
ratio_s2  = T_36 / T_18;              % Stage 2 ratio = 2.0
ratio_s3  = T_20 / T_9;               % Stage 3 ratio = 2.222

% Battery
V_nom    = 12;    C_Ah = 20;   SOC_init = 20;
% Load
P_load   = 5;     % W — LED lamp
% Inverter
P_inv    = 1000;  % W rated (1000W inverter, thesis 4.3.2)
% MPPT setpoint
V_MPPT   = 14.4;  % V DC
% Simulation horizon and timestep (used by all figures)
t_sim    = 300;   dt = 0.5;

%% =========================================================================
%  SECTION 3: WIND SPEED TIME SERIES (thesis Table 3.1)
% =========================================================================

base_wind_vals = [3.0,3.2,3.0,4.0,3.2,3.7,4.2,3.0,4.2,6.0,6.1, ...
                  4.0,3.0,2.9,2.7,3.0,2.7,2.2,4.1,3.0,5.0,3.3, ...
                  3.0,3.1,4.0,2.0,3.0,3.1,4.0];
base_period = 60; % seconds for the 29 recorded readings
base_t = linspace(0, base_period, length(base_wind_vals))';

wind_t = (0:dt:t_sim)';
% Tile the 60s wind record across the full simulation window
wind_v = interp1(base_t, base_wind_vals(:), mod(wind_t, base_period), 'linear');
wind_speed_input = timeseries(wind_v, wind_t);
V_avg = mean(base_wind_vals);
fprintf(' V_avg = %.4f m/s  (%d readings)\n', V_avg, length(base_wind_vals));

% If using multibody inputs and wind_rpm_input not provided, derive from wind speed
if USE_MB_INPUTS && (~exist('wind_rpm_input','var') || isempty(wind_rpm_input))
    wind_rpm_vals = (60/(2*pi*R)) .* wind_v;
    wind_rpm_input = timeseries(wind_rpm_vals, wind_t);
end
% If using multibody inputs and wind_torque_input not provided, derive from wind speed
if USE_MB_INPUTS && (~exist('wind_torque_input','var') || isempty(wind_torque_input))
    wind_torque_vals = 0.5 * rho * Cp * A * (R/TSR) .* (wind_v.^2);
    wind_torque_input = timeseries(wind_torque_vals, wind_t);
end
% Apply viscous load equilibrium if using multibody inputs (prevents runaway RPM)
if USE_MB_INPUTS, wind_rpm_input = timeseries((wind_torque_input.Data ./ B_wind) * (60/(2*pi)), wind_torque_input.Time); end

%% =========================================================================
%  SECTION 4: WAVE RPM — REAL RACK-AND-PINION CALCULATION
%
%  Step 1: Rack linear speed from buoy stroke
%    V_rack = Hs / (T/2)          [upstroke only; same for downstroke]
%  Step 2: Pinion RPM from rack speed
%    rps = V_rack / C_pinion
%    RPM_28T = rps × 60
%  Step 3: Stage 2 — 36T (same shaft) → 18T
%    RPM_74 = RPM_28T × (36/18)
%  Step 4: Stage 3 — 20T → 9T (via flywheel rim)
%    RPM_gen = RPM_74 × (20/9)
%  Add sinusoidal variation (±8%) for realistic wave irregularity
% =========================================================================

V_rack_base  = Hs / (T_w / 2);
RPM_28T_base = (V_rack_base / C_pinion) * 60;
RPM_74_base  = RPM_28T_base * ratio_s2;
RPM_gen_base = RPM_74_base  * ratio_s3;

fprintf('\n GEAR TRAIN RESULTS:\n');
fprintf('   V_rack       = %.4f m/s\n', V_rack_base);
fprintf('   RPM_28T      = %.2f RPM\n', RPM_28T_base);
fprintf('   RPM_after_S2 = %.2f RPM\n', RPM_74_base);
fprintf('   RPM_gen      = %.2f RPM\n', RPM_gen_base);
fprintf('   V_DC_wave    = %.4f V DC\n', RPM_gen_base * k_wave);

% Time-varying RPM with ±8% sinusoidal oscillation at wave frequency
t_vec        = (0:dt:t_sim)';
wave_freq    = 2*pi / T_w;
RPM_variation = 0.08 * RPM_gen_base;
wave_rpm_vals = RPM_gen_base + RPM_variation * sin(wave_freq * t_vec);
if ~USE_MB_INPUTS || ~exist('wave_rpm_input','var') || isempty(wave_rpm_input)
    wave_rpm_input = timeseries(wave_rpm_vals, t_vec);
else
    fprintf(' Using wave_rpm_input from workspace (multibody).\n');
end
% Full-wave rectification: generator output is magnitude of RPM
wave_rpm_input = timeseries(abs(wave_rpm_input.Data), wave_rpm_input.Time);

%% =========================================================================
%  SECTION 5: PRE-COMPUTE REFERENCE VALUES
% =========================================================================

% Wind at average speed
RPM_wind_ref = 60 * (V_avg / (2*pi*R));        % thesis eq 3.10
P_wind_ref   = 0.5 * rho * Cp * A * V_avg^3;
omega_ref    = 2*pi*RPM_wind_ref/60;
T_wind_ref   = P_wind_ref / omega_ref;
V_AC_wind_ref = k_wind * RPM_wind_ref;

% Wave
P_wave_ref    = 0.42 * Hs^2 * T_w * 1000;      % Rodrigues 2008, thesis eq 3.2
V_DC_wave_ref = RPM_gen_base * k_wave;

disp(' ');
disp(' REFERENCE VALUES:');
fprintf('   RPM_wind  = %.2f RPM\n',  RPM_wind_ref);
fprintf('   Torque    = %.4f N.m\n',  T_wind_ref);
fprintf('   V_AC_wind = %.2f V\n',    V_AC_wind_ref);
fprintf('   RPM_gen   = %.2f RPM\n',  RPM_gen_base);
fprintf('   V_DC_wave = %.2f V DC\n', V_DC_wave_ref);
fprintf('   P_wave    = %.2f W/m\n',  P_wave_ref);

%% =========================================================================
%  SECTION 6: BUILD SIMULINK MODEL
% =========================================================================

% Rebuild model each run to avoid stale blocks
mdl = 'hybrid_v4';
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end
new_system(mdl); open_system(mdl);
disp(' Building Simulink model...');

% Helper for block placement
b   = @(cx,cy,w,h) [cx-w/2, cy-h/2, cx+w/2, cy+h/2];
BW  = 120; BH = 56;
Yw  = 100; Yv = 320; Ye = 520;

%% Wind lane
% Two modes:
% - USE_MB_INPUTS = true  -> RPM/torque from multibody
% - USE_MB_INPUTS = false -> RPM/torque computed from wind speed
if USE_MB_INPUTS
    % Wind RPM from workspace (multibody)
    add_block('simulink/Sources/From Workspace',[mdl '/Wind_RPM_in'], ...
        'Position',b(80,Yw,BW,BH),'VariableName','wind_rpm_input', ...
        'SampleTime','0','Interpolate','on');

    % V_AC_wind = k_wind * RPM
    add_block('simulink/Math Operations/Gain',[mdl '/Vac_wind'], ...
        'Position',b(260,Yw,BW,BH),'Gain',sprintf('%.10f',k_wind));

    % Wind Rectifier: V_DC = V_AC * 0.9 * 0.707
    add_block('simulink/Math Operations/Gain',[mdl '/Vdc_wind'], ...
        'Position',b(440,Yw,BW,BH),'Gain',sprintf('%.10f',0.9*0.707));

    % Wind torque from workspace (for logging)
    add_block('simulink/Sources/From Workspace',[mdl '/Wind_Torque_in'], ...
        'Position',b(260,Yw+130,BW,BH),'VariableName','wind_torque_input', ...
        'SampleTime','0','Interpolate','on');
else
    add_block('simulink/Sources/From Workspace',[mdl '/Wind_V'], ...
        'Position',b(80,Yw,BW,BH),'VariableName','wind_speed_input', ...
        'SampleTime','0','Interpolate','on');

    % RPM = 60*V/(2*pi*R)  [thesis eq 3.10]
    add_block('simulink/Math Operations/Gain',[mdl '/RPM_wind'], ...
        'Position',b(260,Yw,BW,BH),'Gain',sprintf('%.10f',60/(2*pi*R)));

    % V_AC_wind = k_wind * RPM
    add_block('simulink/Math Operations/Gain',[mdl '/Vac_wind'], ...
        'Position',b(440,Yw,BW,BH),'Gain',sprintf('%.10f',k_wind));

    % Wind Rectifier: V_DC = V_AC * 0.9 * 0.707
    add_block('simulink/Math Operations/Gain',[mdl '/Vdc_wind'], ...
        'Position',b(620,Yw,BW,BH),'Gain',sprintf('%.10f',0.9*0.707));

    % Wind Power: P = 0.5*rho*Cp*A * V^3
    add_block('simulink/Math Operations/Product',[mdl '/V3'], ...
        'Position',b(260,Yw+130,BW,BH),'Inputs','***');
    add_block('simulink/Math Operations/Gain',[mdl '/P_wind'], ...
        'Position',b(440,Yw+130,BW,BH),'Gain',sprintf('%.10f',0.5*rho*Cp*A));

    % Torque = P / omega
    add_block('simulink/Math Operations/Gain',[mdl '/omega_gain'], ...
        'Position',b(260,Yw+220,BW,BH),'Gain',sprintf('%.10f',2*pi/60));
    add_block('simulink/Math Operations/Product',[mdl '/Torque'], ...
        'Position',b(440,Yw+220,BW,BH),'Inputs','*/');
end

%% Wave lane — real rack-and-pinion output (timeseries)
add_block('simulink/Sources/From Workspace',[mdl '/Wave_RPM_in'], ...
    'Position',b(80,Yv,BW,BH),'VariableName','wave_rpm_input', ...
    'SampleTime','0','Interpolate','on');

% V_DC_wave = k_wave * RPM_gen  (k_wave is already DC)
add_block('simulink/Math Operations/Gain',[mdl '/Vdc_wave'], ...
    'Position',b(260,Yv,BW,BH),'Gain',sprintf('%.10f',k_wave));

%% Electrical
% DC bus combines wind + wave, then MPPT clamps to 14.4V
add_block('simulink/Math Operations/Add',[mdl '/DC_Bus'], ...
    'Position',b(800,Ye,BW,BH),'Inputs','++');
add_block('simulink/Discontinuities/Saturation',[mdl '/MPPT'], ...
    'Position',b(980,Ye,BW,BH),'UpperLimit','14.4','LowerLimit','0');
add_block('simulink/Commonly Used Blocks/Terminator',[mdl '/Load'], ...
    'Position',b(1140,Ye,40,40));

%% Output ports for logging
% Outports let us pull signals from sim_out.yout reliably
add_block('simulink/Sinks/Out1',[mdl '/Out_RPMwind'],  'Position',b(620,Yw-80,BW,BH)); set_param([mdl '/Out_RPMwind'],'Port','1');
add_block('simulink/Sinks/Out1',[mdl '/Out_Torque'],   'Position',b(620,Yw+220,BW,BH)); set_param([mdl '/Out_Torque'],'Port','2');
add_block('simulink/Sinks/Out1',[mdl '/Out_Vacwind'],  'Position',b(800,Yw-80,BW,BH)); set_param([mdl '/Out_Vacwind'],'Port','3');
add_block('simulink/Sinks/Out1',[mdl '/Out_RPMwave'],  'Position',b(440,Yv-80,BW,BH)); set_param([mdl '/Out_RPMwave'],'Port','4');
add_block('simulink/Sinks/Out1',[mdl '/Out_Vdcwave'],  'Position',b(620,Yv-80,BW,BH)); set_param([mdl '/Out_Vdcwave'],'Port','5');
add_block('simulink/Sinks/Out1',[mdl '/Out_VDCbus'],   'Position',b(980,Yw-80,BW,BH)); set_param([mdl '/Out_VDCbus'],'Port','6');

%% Connections
% Wire the signal flow for wind, wave, and electrical lanes
if USE_MB_INPUTS
    add_line(mdl,'Wind_RPM_in/1','Vac_wind/1','autorouting','on');
    add_line(mdl,'Vac_wind/1','Vdc_wind/1','autorouting','on');
    add_line(mdl,'Wind_Torque_in/1','Out_Torque/1','autorouting','on');
    add_line(mdl,'Wind_RPM_in/1','Out_RPMwind/1','autorouting','on');
    add_line(mdl,'Vac_wind/1','Out_Vacwind/1','autorouting','on');
else
    add_line(mdl,'Wind_V/1','RPM_wind/1','autorouting','on');
    add_line(mdl,'Wind_V/1','V3/1','autorouting','on');
    add_line(mdl,'Wind_V/1','V3/2','autorouting','on');
    add_line(mdl,'Wind_V/1','V3/3','autorouting','on');
    add_line(mdl,'RPM_wind/1','Vac_wind/1','autorouting','on');
    add_line(mdl,'Vac_wind/1','Vdc_wind/1','autorouting','on');
    add_line(mdl,'V3/1','P_wind/1','autorouting','on');
    add_line(mdl,'RPM_wind/1','omega_gain/1','autorouting','on');
    add_line(mdl,'P_wind/1','Torque/1','autorouting','on');
    add_line(mdl,'omega_gain/1','Torque/2','autorouting','on');
    add_line(mdl,'RPM_wind/1','Out_RPMwind/1','autorouting','on');
    add_line(mdl,'Torque/1','Out_Torque/1','autorouting','on');
    add_line(mdl,'Vac_wind/1','Out_Vacwind/1','autorouting','on');
end

add_line(mdl,'Wave_RPM_in/1','Vdc_wave/1','autorouting','on');

add_line(mdl,'Vdc_wind/1','DC_Bus/1','autorouting','on');
add_line(mdl,'Vdc_wave/1','DC_Bus/2','autorouting','on');
add_line(mdl,'DC_Bus/1','MPPT/1','autorouting','on');
add_line(mdl,'MPPT/1','Load/1','autorouting','on');

add_line(mdl,'Wave_RPM_in/1','Out_RPMwave/1','autorouting','on');
add_line(mdl,'Vdc_wave/1','Out_Vdcwave/1','autorouting','on');
add_line(mdl,'DC_Bus/1','Out_VDCbus/1','autorouting','on');

%% Sim settings
% Fixed step for smooth plots; yout/tout returned for robust logging
set_param(mdl,'Solver','ode45','StopTime',sprintf('%.1f',t_sim), ...
    'MaxStep',sprintf('%.2f',dt),'SaveOutput','on','OutputSaveName','yout', ...
    'SaveTime','on','TimeSaveName','tout','ReturnWorkspaceOutputs','on');

set_param(mdl,'ZoomFactor','FitSystem');
save_system(mdl,[mdl '.slx']);
disp(' Running simulation...');
sim_out = sim(mdl,'StopTime',sprintf('%.1f',t_sim));
disp(' Simulation complete.');

%% =========================================================================
%  SECTION 7: EXTRACT DATA
% =========================================================================
% yout can be numeric, struct, or Dataset depending on MATLAB version

t    = sim_out.tout;
yout = sim_out.yout;

% Robust extraction (numeric / struct / dataset)
if isstruct(yout) && isfield(yout,'signals')
    wind_rpm = yout.signals(1).values;
    torque   = yout.signals(2).values;
    vac_wind = yout.signals(3).values;
    wave_rpm = yout.signals(4).values;
    vdc_wave = yout.signals(5).values;
    vdc_bus  = yout.signals(6).values;
elseif isa(yout,'Simulink.SimulationData.Dataset')
    wind_rpm = yout.getElement(1).Values.Data;
    torque   = yout.getElement(2).Values.Data;
    vac_wind = yout.getElement(3).Values.Data;
    wave_rpm = yout.getElement(4).Values.Data;
    vdc_wave = yout.getElement(5).Values.Data;
    vdc_bus  = yout.getElement(6).Values.Data;
else
    wind_rpm = yout(:,1);
    torque   = yout(:,2);
    vac_wind = yout(:,3);
    wave_rpm = yout(:,4);
    vdc_wave = yout(:,5);
    vdc_bus  = yout(:,6);
end

fprintf(' Data rows: %d  (t=%.1f to %.1f s)\n',length(t),t(1),t(end));

%% =========================================================================
%  SECTION 8: SOC — computed in MATLAB (cumtrapz)
%  P_total = P_wind(t) + P_wave(constant)
%  dSOC/dt = P_net / (C_Ah * V_nom * 3600) * 100  [%/s]
% =========================================================================

P_wave_const = 0.42 * Hs^2 * T_w * 1000;
if USE_MB_INPUTS
    omega_wind = wind_rpm * (2*pi/60);
    P_wind_arr = torque .* omega_wind;
else
    P_wind_arr = 0.5 * rho * Cp * A .* (wind_rpm * R * (2*pi/60) / TSR).^3;
end
P_total      = P_wind_arr + P_wave_const;
P_net        = max(P_total - P_load, 0);
SOC_rate     = P_net / (C_Ah * V_nom * 3600) * 100;
delta_SOC    = cumtrapz(t, SOC_rate);
soc          = min(SOC_init + delta_SOC, 100);
fprintf(' SOC: %.1f%% -> %.1f%%  (Delta=+%.1f%%)\n',soc(1),soc(end),soc(end)-soc(1));

% Figure-specific time windows (keep all figures consistent)
t_fig_wind = t_sim;
t_fig_elec = t_sim;
t_fig_soc  = t_sim;

%% =========================================================================
%  SECTION 9: OUTPUT DIRECTORIES
% =========================================================================
outdir_fig = fullfile(pwd,'outputs','figures');
outdir_csv = fullfile(pwd,'outputs','csv');
if ~exist(outdir_fig,'dir'), mkdir(outdir_fig); end
if ~exist(outdir_csv,'dir'), mkdir(outdir_csv); end

%% =========================================================================
%  SECTION 10: FIGURES
% =========================================================================

disp(' Generating figures...');

%% Figure 4.1 — Wind RPM and Torque
f1=figure('Name','Fig4.1','NumberTitle','off','Color','k','Position',[50,500,920,540]);
ax=subplot(2,1,1); plot(t,wind_rpm,'Color',[0.94 0.65 0.0],'LineWidth',2);
set(ax,'Color','k','XColor','w','YColor','w','GridColor',[0.25 0.25 0.25]);
xlabel('Time (s)','Color','w'); ylabel('RPM','Color','w');
title('Wind Turbine Rotational Speed (RPM)','Color','w','FontSize',11);
xlim([0 t_fig_wind]); ylim([0 max(wind_rpm)*1.2]); grid on; grid minor;
legend('RPM_{wind}','TextColor','w','Color','none','EdgeColor','w');

ax=subplot(2,1,2); plot(t,torque,'Color',[0.94 0.65 0.0],'LineWidth',2);
set(ax,'Color','k','XColor','w','YColor','w','GridColor',[0.25 0.25 0.25]);
xlabel('Time (s)','Color','w'); ylabel('Torque (N·m)','Color','w');
title('Wind Turbine Shaft Torque','Color','w','FontSize',11);
xlim([0 t_fig_wind]); grid on; grid minor;
legend('Torque_{wind}','TextColor','w','Color','none','EdgeColor','w');

sgtitle({sprintf('Figure 4.1 — Wind Turbine Output  |  %s',label), ...
    sprintf('VAWT: H=%.2fm D=%.2fm Cp=%.2f TSR=%.1f  [RPM=60V/2πR, Thesis Eq.3.10]', ...
    H,D,Cp,TSR)},'Color','w','FontSize',11,'FontWeight','bold');
fn=fullfile(outdir_fig,sprintf('Fig4_1_WindRPM_%s.png',suffix));
exportgraphics(f1,fn,'Resolution',200,'BackgroundColor','k');
fprintf(' Saved: %s\n',fn);

%% Figure 4.2 — Wave RPM and V_DC_wave
f2=figure('Name','Fig4.2','NumberTitle','off','Color','k','Position',[80,450,920,540]);
ax=subplot(2,1,1); plot(t,wave_rpm,'Color',[0.0 0.74 0.84],'LineWidth',2);
set(ax,'Color','k','XColor','w','YColor','w','GridColor',[0.25 0.25 0.25]);
xlabel('Time (s)','Color','w'); ylabel('RPM','Color','w');
title('Wave Generator Shaft Speed (RPM)','Color','w','FontSize',11);
xlim([0 t_sim]); ylim([RPM_gen_base*0.8 RPM_gen_base*1.2]); grid on; grid minor;
yline(RPM_gen_base,'b--','LineWidth',1.5,'Label', ...
    sprintf('RPM_{base}=%.1f',RPM_gen_base),'LabelHorizontalAlignment','left');
legend('RPM_{wave}','TextColor','w','Color','none','EdgeColor','w');

ax=subplot(2,1,2); plot(t,vdc_wave,'Color',[0.0 0.74 0.84],'LineWidth',2);
set(ax,'Color','k','XColor','w','YColor','w','GridColor',[0.25 0.25 0.25]);
xlabel('Time (s)','Color','w'); ylabel('V DC (V)','Color','w');
title('Wave Generator Output Voltage (V_{DC,wave})','Color','w','FontSize',11);
xlim([0 t_fig_elec]); grid on; grid minor;
yline(12,'r--','LineWidth',1.5,'Label','12V Battery Threshold', ...
    'LabelHorizontalAlignment','left');
legend('V_{DC,wave}','TextColor','w','Color','none','EdgeColor','w');

sgtitle({sprintf('Figure 4.2 — Wave Generator Output  |  %s',label), ...
    sprintf('Rack→28T→36T/18T→20T/9T | k=0.125V/RPM | RPM_{base}=%.1f',RPM_gen_base)}, ...
    'Color','w','FontSize',11,'FontWeight','bold');
fn=fullfile(outdir_fig,sprintf('Fig4_2_WaveRPM_%s.png',suffix));
exportgraphics(f2,fn,'Resolution',200,'BackgroundColor','k');
fprintf(' Saved: %s\n',fn);

%% Figure 4.3 — Electrical Voltages
f3=figure('Name','Fig4.3','NumberTitle','off','Color','k','Position',[110,400,920,660]);
ax=subplot(3,1,1); plot(t,vac_wind,'Color',[0.94 0.65 0.0],'LineWidth',2);
set(ax,'Color','k','XColor','w','YColor','w','GridColor',[0.25 0.25 0.25]);
xlabel('Time (s)','Color','w'); ylabel('V AC (V)','Color','w');
title('Wind Generator Output Voltage (V_{AC,wind})','Color','w','FontSize',11);
xlim([0 t_fig_elec]); grid on; grid minor;
legend('V_{AC,wind}','TextColor','w','Color','none','EdgeColor','w');

ax=subplot(3,1,2); plot(t,vdc_wave,'Color',[0.0 0.74 0.84],'LineWidth',2);
set(ax,'Color','k','XColor','w','YColor','w','GridColor',[0.25 0.25 0.25]);
xlabel('Time (s)','Color','w'); ylabel('V DC (V)','Color','w');
title('Wave Generator Output Voltage (V_{DC,wave})','Color','w','FontSize',11);
xlim([0 t_fig_elec]); grid on; grid minor;
legend('V_{DC,wave}','TextColor','w','Color','none','EdgeColor','w');

ax=subplot(3,1,3); plot(t,vdc_bus,'Color',[0.47 1.0 0.01],'LineWidth',2);
set(ax,'Color','k','XColor','w','YColor','w','GridColor',[0.25 0.25 0.25]);
xlabel('Time (s)','Color','w'); ylabel('V DC (V)','Color','w');
title('Combined DC Bus Voltage','Color','w','FontSize',11);
xlim([0 t_sim]); grid on; grid minor;
yline(12,  'r--','LineWidth',1.5,'Label','12V Battery','LabelHorizontalAlignment','left');
yline(14.4,'g--','LineWidth',1.5,'Label','14.4V MPPT', 'LabelHorizontalAlignment','left');
legend('V_{DC,bus}','TextColor','w','Color','none','EdgeColor','w');

sgtitle(sprintf('Figure 4.3 — Electrical Output Voltages  |  %s',label), ...
    'Color','w','FontSize',11,'FontWeight','bold');
fn=fullfile(outdir_fig,sprintf('Fig4_3_Electrical_%s.png',suffix));
exportgraphics(f3,fn,'Resolution',200,'BackgroundColor','k');
fprintf(' Saved: %s\n',fn);

%% Figure 4.4 — Battery SOC
f4=figure('Name','Fig4.4','NumberTitle','off','Color','k','Position',[140,350,920,440]);
plot(t,soc,'Color',[0.88 0.25 0.98],'LineWidth',2.5);
ax=gca; set(ax,'Color','k','XColor','w','YColor','w','GridColor',[0.25 0.25 0.25]);
xlabel('Time (s)','Color','w'); ylabel('SOC (%)','Color','w');
title('Battery State of Charge (SOC) Over Time','Color','w','FontSize',11);
xlim([0 t_fig_soc]); ylim([0 100]); grid on; grid minor;
yline(80,'g--','LineWidth',1.5,'Label','80% Full','LabelHorizontalAlignment','left');
yline(20,'r--','LineWidth',1.5,'Label','20% Low', 'LabelHorizontalAlignment','left');
legend('SOC (%)','TextColor','w','Color','none','EdgeColor','w');

sgtitle({sprintf('Figure 4.4 — Battery Charging Profile  |  %s',label), ...
    sprintf('12V/%.0fAh | P_{load}=%.0fW | SOC: %.1f%% → %.1f%%', ...
    C_Ah,P_load,soc(1),soc(end))},'Color','w','FontSize',11,'FontWeight','bold');
fn=fullfile(outdir_fig,sprintf('Fig4_4_BatterySOC_%s.png',suffix));
exportgraphics(f4,fn,'Resolution',200,'BackgroundColor','k');
fprintf(' Saved: %s\n',fn);

%% =========================================================================
%  SECTION 11: CSV EXPORT
% =========================================================================
csv_file=fullfile(outdir_csv,sprintf('simulation_results_%s.csv',suffix));
fid=fopen(csv_file,'w');
fprintf(fid,'time_s,wind_rpm,wind_torque_Nm,wind_vac_V,wave_rpm,wave_vdc_V,vdc_bus_V,soc_pct\n');
for i=1:length(t)
    fprintf(fid,'%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
        t(i),wind_rpm(i),torque(i),vac_wind(i), ...
        wave_rpm(i),vdc_wave(i),vdc_bus(i),soc(i));
end
fclose(fid);
fprintf(' Saved: %s\n',csv_file);

%% =========================================================================
%  SECTION 12: CONSOLE SUMMARY
% =========================================================================
try
    disp(' ');
    disp('=============================================================');
    disp(' RESULTS SUMMARY - REAL PARAMETERS');
    disp('=============================================================');
    fprintf(' Tide   : %s\n',label);
    fprintf(' Points : %d  (%.1f-%.1f s)\n',length(t),t(1),t(end));
    fprintf('\n GEAR TRAIN (Rack-and-Pinion, 3-Stage):\n');
    fprintf('   Pinion  : 28T, C=%.4fm\n',C_pinion);
    fprintf('   Stage 2 : 36T->18T (ratio %.1f)\n',ratio_s2);
    fprintf('   Stage 3 : 20T->9T  (ratio %.4f)\n',ratio_s3);
    fprintf('   V_rack  : %.4f m/s\n',V_rack_base);
    fprintf('   RPM_gen : %.2f RPM\n',RPM_gen_base);
    fprintf('\n WIND TURBINE:\n');
    fprintf('   RPM : avg=%.2f  min=%.2f  max=%.2f\n',mean(wind_rpm),min(wind_rpm),max(wind_rpm));
    fprintf('   Torque: avg=%.4f  max=%.4f N.m\n',mean(torque),max(torque));
    fprintf('   V_AC: avg=%.2f  max=%.2f V\n',mean(vac_wind),max(vac_wind));
    fprintf('\n WAVE GENERATOR:\n');
    fprintf('   RPM : avg=%.2f  min=%.2f  max=%.2f\n',mean(wave_rpm),min(wave_rpm),max(wave_rpm));
    fprintf('   V_DC: avg=%.2f  min=%.2f  max=%.2f V\n',mean(vdc_wave),min(vdc_wave),max(vdc_wave));
    fprintf('\n ELECTRICAL:\n');
    fprintf('   V_DC_bus: avg=%.2f  min=%.2f  max=%.2f V\n',mean(vdc_bus),min(vdc_bus),max(vdc_bus));
    fprintf('   Above 12V?  %s\n',ternary(min(vdc_bus)>12,'YES','NO'));
    fprintf('   Above 14.4V? %s\n',ternary(min(vdc_bus)>14.4,'YES','PARTIAL'));
    fprintf('\n BATTERY:\n');
    fprintf('   SOC: %.1f%% -> %.1f%%  (Delta=+%.1f%%)\n',soc(1),soc(end),soc(end)-soc(1));
    disp('=============================================================');
    fprintf(' Change TIDE_MODE=''low'' and re-run for Low Tide\n');
    disp('=============================================================');
catch
    % Avoid batch run failure if output stream is unavailable.
end

function r = ternary(cond,a,b)
    if cond, r=a; else, r=b; end
end
