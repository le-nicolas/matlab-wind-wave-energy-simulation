%% RUN_FULL_SYSTEM_MULTIBODY.M
% Full-system run: Multibody -> Electrical (Hybrid v4)
% Exports multibody traces and then runs build_hybrid_model_v4 using
% multibody RPM inputs for wind and wave.

clear; clc;

tide_list = {'high','low'};

for k = 1:numel(tide_list)
    TIDE_MODE = tide_list{k};
    clear ts_buoy_z
    USE_RACK_PINION_CONSTRAINT = true;
    WAVE_DRIVE_MODE = 'motion';
    if strcmp(TIDE_MODE,'high')
        suffix = 'HT';
    else
        suffix = 'LT';
    end

    % Build multibody model (no animation/video in headless runs)
    DO_ANIMATE = false;
    DO_VIDEO   = false;
    MODEL_SUFFIX = suffix;
    % Suppress console output in batch runs to avoid stream errors
    evalc("run('build_multibody_fixed.m');");

    % Simulate multibody model for traces
    mdl_mb = sprintf('buoy_multibody_fixed_%s', suffix);
    if ~bdIsLoaded(mdl_mb)
        load_system(mdl_mb);
    end

    set_param(mdl_mb, 'SaveOutput','on','OutputSaveName','yout', ...
        'SaveTime','on','TimeSaveName','tout','ReturnWorkspaceOutputs','on');

    used_fallback = false;
    sim_err = '';
    try
        sim_out_mb = sim(mdl_mb);
        [t_mb, y_mb] = local_extract(sim_out_mb);

        % Try to pull buoy Z timeseries from sim output if present
        try
            ts_buoy_z = sim_out_mb.get('ts_buoy_z');
        catch
        end

        % Port order: 1 Wave RPM, 2 Wave Torque, 3 Wind RPM, 4 Wind Torque, 5 VDC bus
        wave_rpm   = y_mb(:,1);
        wave_torque= y_mb(:,2);
        wind_rpm   = y_mb(:,3);
        wind_torque= y_mb(:,4);
        vdc_bus    = y_mb(:,5);

        % If wave RPM is all zeros (multibody coupling not effective), fall back to kinematic wave
        if all(abs(wave_rpm) < 1e-6)
            used_fallback = true;
            sim_err = 'Wave RPM zero from multibody; using kinematic wave fallback.';

            C_pinion = T_pinion * pitch;
            V_rack_base  = Hs / (T_w / 2);
            RPM_28T_base = (V_rack_base / C_pinion) * 60;
            RPM_36_base  = RPM_28T_base * ratio_s2;
            RPM_gen_base = RPM_36_base  * ratio_s3;
            wave_variation = 0.08 * RPM_gen_base;
            wave_rpm = RPM_gen_base + wave_variation * sin(2*pi/T_w * t_mb);

            P_wave_const = 0.42 * Hs^2 * T_w * 1000;
            omega_wave = wave_rpm * (2*pi/60);
            wave_torque = P_wave_const ./ max(omega_wave, 1e-6);

            % Use kinematic wind RPM from wind speed series for consistency
            wind_rpm_vals = (60/(2*pi*R)) .* wind_vals;
            wind_rpm = interp1(wind_t, wind_rpm_vals, t_mb, 'linear', 'extrap');
            wind_torque = interp1(wind_t, wind_torque_vals, t_mb, 'linear', 'extrap');

            vdc_wind = min(k_wind * wind_rpm * (0.9*0.707), 14.4);
            vdc_wave = min(k_wave * wave_rpm, 14.4);
            vdc_bus  = vdc_wind + vdc_wave;
        end
    catch ME
        sim_err = ME.message;
        try
            fprintf(2, '\n--- Multibody sim error details (%s) ---\n%s\n', ...
                TIDE_MODE, ME.getReport('extended','hyperlinks','off'));
        catch
        end

        % Retry once with kinematic coupling (disable rack/pinion constraint)
        retried = false;
        if USE_RACK_PINION_CONSTRAINT
            USE_RACK_PINION_CONSTRAINT = false;
            WAVE_DRIVE_MODE = 'motion';
            MODEL_SUFFIX = suffix;
            evalc("run('build_multibody_fixed.m');");
            retried = true;

            mdl_mb = sprintf('buoy_multibody_fixed_%s', suffix);
            if ~bdIsLoaded(mdl_mb)
                load_system(mdl_mb);
            end
            set_param(mdl_mb, 'SaveOutput','on','OutputSaveName','yout', ...
                'SaveTime','on','TimeSaveName','tout','ReturnWorkspaceOutputs','on');
            try
                sim_out_mb = sim(mdl_mb);
                [t_mb, y_mb] = local_extract(sim_out_mb);

                % Pull buoy Z timeseries if present
                try
                    ts_buoy_z = sim_out_mb.get('ts_buoy_z');
                catch
                end

                wave_rpm   = y_mb(:,1);
                wave_torque= y_mb(:,2);
                wind_rpm   = y_mb(:,3);
                wind_torque= y_mb(:,4);
                vdc_bus    = y_mb(:,5);
            catch ME2
                sim_err = ME2.message;
                used_fallback = true;
            end
        else
            used_fallback = true;
        end

        if ~retried || used_fallback
            used_fallback = true;

            % Fallback: kinematic wave RPM + wind RPM from speed series
            t_mb = (0:0.1:t_sim)';

        % Wave RPM kinematic approximation (rack-pinion)
        C_pinion = T_pinion * pitch;
        V_rack_base  = Hs / (T_w / 2);
        RPM_28T_base = (V_rack_base / C_pinion) * 60;
        RPM_36_base  = RPM_28T_base * ratio_s2;
        RPM_gen_base = RPM_36_base  * ratio_s3;
        wave_variation = 0.08 * RPM_gen_base;
        wave_rpm = RPM_gen_base + wave_variation * sin(2*pi/T_w * t_mb);

        % Wave torque from constant wave power
        P_wave_const = 0.42 * Hs^2 * T_w * 1000;
        omega_wave = wave_rpm * (2*pi/60);
        wave_torque = P_wave_const ./ max(omega_wave, 1e-6);

        % Wind RPM + torque from wind speed series
        wind_rpm_vals = (60/(2*pi*R)) .* wind_vals;
        wind_rpm = interp1(wind_t, wind_rpm_vals, t_mb, 'linear', 'extrap');
        wind_torque = interp1(wind_t, wind_torque_vals, t_mb, 'linear', 'extrap');

            % DC bus from electrical gains (per-source MPPT)
            vdc_wind = min(k_wind * wind_rpm * (0.9*0.707), 14.4);
            vdc_wave = min(k_wave * wave_rpm, 14.4);
            vdc_bus  = vdc_wind + vdc_wave;
        end
    end

    %% Export multibody traces
    outdir_csv = fullfile(pwd,'outputs','csv');
    outdir_fig = fullfile(pwd,'outputs','figures');
    if ~exist(outdir_csv,'dir'), mkdir(outdir_csv); end
    if ~exist(outdir_fig,'dir'), mkdir(outdir_fig); end

    csv_file = fullfile(outdir_csv, sprintf('multibody_traces_%s.csv', suffix));
    fid = fopen(csv_file,'w');
    fprintf(fid,'time_s,wave_rpm,wave_torque_Nm,wind_rpm,wind_torque_Nm,vdc_bus_V\n');
    for i = 1:numel(t_mb)
        fprintf(fid,'%.4f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
            t_mb(i), wave_rpm(i), wave_torque(i), wind_rpm(i), wind_torque(i), vdc_bus(i));
    end
    fclose(fid);

    f = figure('Name',sprintf('Multibody Traces %s',suffix),'Color','k', ...
        'Position',[120 180 980 620]);

    subplot(3,1,1);
    plot(t_mb, wave_rpm,'Color',[0.0 0.74 0.84],'LineWidth',1.8);
    yyaxis right;
    plot(t_mb, wave_torque,'Color',[0.94 0.65 0.0],'LineWidth',1.2);
    set(gca,'Color','k','XColor','w','YColor','w');
    xlabel('Time (s)','Color','w'); ylabel('Wave RPM / Torque','Color','w');
    title('Wave Shaft: RPM (left) and Torque (right)','Color','w');
    grid on; grid minor; xlim([0 t_mb(end)]);

    subplot(3,1,2);
    plot(t_mb, wind_rpm,'Color',[0.47 1.0 0.01],'LineWidth',1.8);
    yyaxis right;
    plot(t_mb, wind_torque,'Color',[0.88 0.25 0.98],'LineWidth',1.2);
    set(gca,'Color','k','XColor','w','YColor','w');
    xlabel('Time (s)','Color','w'); ylabel('Wind RPM / Torque','Color','w');
    title('Wind Shaft: RPM (left) and Torque (right)','Color','w');
    grid on; grid minor; xlim([0 t_mb(end)]);

    subplot(3,1,3);
    plot(t_mb, vdc_bus,'Color',[0.2 0.8 1.0],'LineWidth',1.8);
    set(gca,'Color','k','XColor','w','YColor','w');
    xlabel('Time (s)','Color','w'); ylabel('Vdc (V)','Color','w');
    title('DC Bus Voltage (Multibody Electrical Combine)','Color','w');
    grid on; grid minor; xlim([0 t_mb(end)]);

    fn = fullfile(outdir_fig, sprintf('Multibody_Traces_%s.png', suffix));
    exportgraphics(f, fn, 'Resolution',200, 'BackgroundColor','k');
    close(f);

    if used_fallback
        fprintf('Fallback used for multibody traces (%s).\n', sim_err);
    end

    % Export buoy Z displacement if available (diagnostic)
    if exist('ts_buoy_z','var') && ~isempty(ts_buoy_z)
        try
            t_buoy = ts_buoy_z.Time;
            z_buoy = ts_buoy_z.Data;
            outdir_csv = fullfile(pwd,'outputs','csv');
            if ~exist(outdir_csv,'dir'), mkdir(outdir_csv); end
            buoy_csv = fullfile(outdir_csv, sprintf('buoy_z_%s.csv', suffix));
            fid = fopen(buoy_csv,'w');
            fprintf(fid,'time_s,buoy_z_m\n');
            for ii = 1:numel(t_buoy)
                fprintf(fid,'%.4f,%.6f\n', t_buoy(ii), z_buoy(ii));
            end
            fclose(fid);
        catch
        end
    end

    %% Feed multibody RPM into hybrid electrical model
    wave_rpm_input   = timeseries(wave_rpm, t_mb);
    wind_rpm_input   = timeseries(wind_rpm, t_mb);
    wind_torque_input= timeseries(wind_torque, t_mb);

    USE_MB_INPUTS = true;
    % Suppress console output in batch runs to avoid stream errors
    evalc("run('build_hybrid_model_v4.m');");

    % Restore loop data if cleared by downstream scripts
    tide_list = {'high','low'};
end

%% -------- Local helper: extract yout safely ------------------------------
function [t, y] = local_extract(sim_out)
    t = sim_out.tout;
    yout = sim_out.yout;
    if isa(yout, 'Simulink.SimulationData.Dataset')
        n = yout.numElements;
        y = zeros(numel(t), n);
        for i = 1:n
            sig = yout.get(i);
            y(:,i) = sig.Values.Data;
        end
    elseif isstruct(yout) && isfield(yout,'signals')
        n = numel(yout.signals);
        y = zeros(numel(t), n);
        for i = 1:n
            y(:,i) = yout.signals(i).values;
        end
    else
        y = yout;
    end
    t = t(:);
    y = y(:,:);
end
