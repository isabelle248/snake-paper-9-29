%paraclc
close all
set(groot, 'DefaultFigureWindowStyle', 'docked');


% Fixed baseline values
% amplitude_base = 0.05;
% frequency_base = 1.0;
% wavelength_base = 1;
% phase_offset_base = pi/2;

% ================================================================
% CHOOSE PARAMETER TO SWEEP
% ================================================================

sweep_parameter = "wavelength";
% Options:
% "amplitude"
% "frequency"
% "wavelength"

% Fixed baseline values
amplitude_base = 1.53;
frequency_base = 1.825;

wavelength_base = 0.4;
phase_offset_base = pi/2;

% Parameter ranges
amplitude_vals = linspace(0.1, 1.80, 20);
frequency_vals = linspace(1, 2.65, 20);
wavelength_vals = linspace(0.5, 0.55, 1);

amplitude_vals = round(amplitude_vals, 2);
frequency_vals = round(frequency_vals, 2);
wavelength_vals = round(wavelength_vals, 2);


% ================================================================
% SELECT SWEEP VALUES
% ================================================================

switch sweep_parameter

    case "amplitude"
        parameter_vals = amplitude_vals;
        parameter_name = 'Amplitude';
        fprintf('sweep param: %s\n', sweep_parameter);

    case "frequency"
        parameter_vals = frequency_vals;
        parameter_name = 'Frequency';

    case "wavelength"
        parameter_vals = wavelength_vals;
        parameter_name = 'Wavelength';

    otherwise
        error('Invalid sweep parameter.');
end

n = length(parameter_vals);


% AMPLITUDE
% LL_amp = zeros(size(amplitude_vals));
% LA_amp = zeros(size(amplitude_vals));
% AL_amp = zeros(size(amplitude_vals));
% AA_amp = zeros(size(amplitude_vals));
% 
% parameter = amplitude_vals;
% 
% for i = 1:length(amplitude_vals)
% 
%     p = parameter(i)
% 
%     % amplitude
%     LL_amp(i) = custom_main_snake(p, frequency_base, wavelength_base, phase_offset_base, 1, 1);
%     LA_amp(i) = custom_main_snake(p, frequency_base, wavelength_base, phase_offset_base, 1, 2);
%     AL_amp(i) = custom_main_snake(p, frequency_base, wavelength_base, phase_offset_base, 2, 1);
%     AA_amp(i) = custom_main_snake(p, frequency_base, wavelength_base, phase_offset_base, 2, 2);
% end

% One row for each simulation
parameter_col = zeros(4*n,1);

loco_col = strings(4*n,1);
env_col = strings(4*n,1);

forward_disp_col = zeros(4*n,1);
total_disp_col = zeros(4*n,1);
lateral_col = zeros(4*n,1);
yaw_col = zeros(4*n,1);
speed_col = zeros(4*n,1);
cycle_col = zeros(4*n,1);


row = 1;

for i = 1:n

    p = parameter_vals(i);

    switch sweep_parameter

        case "amplitude"
            amp = p;
            freq = frequency_base;
            wavelen = wavelength_base;

        case "frequency"
            amp = amplitude_base;
            freq = p;
            wavelen = wavelength_base;

        case "wavelength"
            amp = amplitude_base;
            freq = frequency_base;
            wavelen = p;

    end
    % 
    % ============================================================
    % LAND LOCOMOTION + LAND ENVIRONMENT
    % ============================================================
    % % % 
    % [forward_disp, total_disp, lateral, yaw, speed, cycle] = ...
    %     custom_main_snake( ...
    %     amp, freq, wavelen, phase_offset_base, 1, 1);
    % 
    % parameter_col(row) = p;
    % loco_col(row) = "Land";
    % env_col(row) = "Land";
    % 
    % forward_disp_col(row) = forward_disp;
    % total_disp_col(row) = total_disp;
    % lateral_col(row) = lateral;
    % yaw_col(row) = yaw;
    % speed_col(row) = speed;
    % cycle_col(row) = cycle;
    % 
    % row = row + 1;
    % fprintf('LAND LOCOMOTION + LAND ENVIRONMENT \n');
    % 

    % ============================================================
    % % LAND LOCOMOTION + AQUATIC ENVIRONMENT
    % % ============================================================
    % 
    % [forward_disp, total_disp, lateral, yaw, speed, cycle] = ...
    %     custom_main_snake( ...
    %     amp, freq, wavelen, phase_offset_base, 1, 2);
    % 
    % parameter_col(row) = p;
    % loco_col(row) = "Land";
    % env_col(row) = "Aquatic";
    % 
    % forward_disp_col(row) = forward_disp;
    % total_disp_col(row) = total_disp;
    % lateral_col(row) = lateral;
    % yaw_col(row) = yaw;
    % speed_col(row) = speed;
    % cycle_col(row) = cycle;
    % 
    % row = row + 1;
    % fprintf('LAND LOCOMOTION + AQUATIC ENVIRONMENT \n');
    % 
    % 
    % % % ============================================================
    % % AQUATIC LOCOMOTION + LAND ENVIRONMENT
    % % ============================================================
    % 
    % [forward_disp, total_disp, lateral, yaw, speed, cycle] = ...
    %     custom_main_snake( ...
    %     amp, freq, wavelen, phase_offset_base, 2, 1);
    % 
    % parameter_col(row) = p;
    % loco_col(row) = "Aquatic";
    % env_col(row) = "Land";
    % 
    % forward_disp_col(row) = forward_disp;
    % total_disp_col(row) = total_disp;
    % lateral_col(row) = lateral;
    % yaw_col(row) = yaw;
    % speed_col(row) = speed;
    % cycle_col(row) = cycle;
    % 
    % row = row + 1;
    % fprintf('AQUATIC LOCOMOTION + LAND ENVIRONMENT \n');

    % 
    % 
    % % ============================================================
    % % AQUATIC LOCOMOTION + AQUATIC ENVIRONMENT
    % % ============================================================
    % 
    [forward_disp, total_disp, lateral, yaw, speed, cycle] = ...
        custom_main_snake( ...
        amp, freq, wavelen, phase_offset_base, 2, 2);

    parameter_col(row) = p;
    loco_col(row) = "Aquatic";
    env_col(row) = "Aquatic";

    forward_disp_col(row) = forward_disp;
    total_disp_col(row) = total_disp;
    lateral_col(row) = lateral;
    yaw_col(row) = yaw;
    speed_col(row) = speed;
    cycle_col(row) = cycle;

    row = row + 1;
    fprintf('AQUATIC LOCOMOTION + AQUATIC ENVIRONMENT \n');

end
% ================================================================
% CREATE AND SAVE TABLE
% ================================================================
sweep_parameter_col = repmat(sweep_parameter, 4*n, 1);

metrics_summary = table( ...
    sweep_parameter_col, ...
    parameter_col, ...
    loco_col, ...
    env_col, ...
    forward_disp_col, ...
    total_disp_col, ...
    lateral_col, ...
    yaw_col, ...
    speed_col, ...
    cycle_col, ...
    'VariableNames', { ...
    'SweepParameter', ...
    'SweepValue', ...
    'LocoType', ...
    'EnvType', ...
    'ForwardDisp', ...
    'TotalDisp', ...
    'LateralDrift', ...
    'NetYaw', ...
    'MeanSpeedSteady', ...
    'DistancePerCycle'});

% Save table as CSV
writetable(metrics_summary, sweep_parameter + '_metrics.csv');

% Display table in MATLAB
disp(metrics_summary);

% ================================================================
% PLOT SIGNED FORWARD DISPLACEMENT
% ================================================================


figure;
hold on;
plot(parameter_vals, ...
    forward_disp_col(loco_col == "Land" & env_col == "Land"), ...
    '.', 'DisplayName', 'Land Loco + Land Env', 'LineWidth',2, 'MarkerSize', 15);

plot(parameter_vals, ...
    forward_disp_col(loco_col == "Land" & env_col == "Aquatic"), ...
    '.', 'DisplayName', 'Land Loco + Aquatic Env', 'LineWidth',2, 'MarkerSize', 15);

plot(parameter_vals, ...
    forward_disp_col(loco_col == "Aquatic" & env_col == "Land"), ...
    '.', 'DisplayName', 'Aquatic Loco + Land Env', 'LineWidth',2, 'MarkerSize', 15);

plot(parameter_vals, ...
    forward_disp_col(loco_col == "Aquatic" & env_col == "Aquatic"), ...
    '.', 'DisplayName', 'Aquatic Loco + Aquatic Env', 'LineWidth',2, 'MarkerSize', 15);

yline(0, '--');

xlabel(parameter_name);
ylabel('Signed Forward Displacement (m)');
title(['Forward Displacement vs ' parameter_name]);
legend('Location', 'best');
grid on;
hold off;

% % % FREQUENCY
% LL_freq = zeros(size(number_vals));
% LA_freq = zeros(size(number_vals));
% AL_freq = zeros(size(number_vals));
% AA_freq = zeros(size(number_vals));
% 
% parameter = frequency_vals;
% 
% for i = 1:length(frequency_vals)
% 
%     p = parameter(i);
% 
%     % frequency
%     LL_freq(i) = custom_main_snake(amplitude_base, p, wavelength_base, phase_offset_base, 1, 1);
%     LA_freq(i) = custom_main_snake(amplitude_base, p, wavelength_base, phase_offset_base, 1, 2);
%     AL_freq(i) = custom_main_snake(amplitude_base, p, wavelength_base, phase_offset_base, 2, 1);
%     AA_freq(i) = custom_main_snake(amplitude_base, p, wavelength_base, phase_offset_base, 2, 2);
% end
% 
% %WAVELENGTH
% LL_w = zeros(size(number_vals));
% LA_w = zeros(size(number_vals));
% AL_w = zeros(size(number_vals));
% AA_w = zeros(size(number_vals));
% 
% parameter = wavelength_vals;
% 
% for i = 1:length(wavelength_vals)
% 
%     p = parameter(i);
% 
%     LL_w(i) = custom_main_snake(amplitude_base, frequency_base, p, phase_offset_base, 1, 1);
%     LA_w(i) = custom_main_snake(amplitude_base, frequency_base, p, phase_offset_base, 1, 2);
%     AL_w(i) = custom_main_snake(amplitude_base, frequency_base, p, phase_offset_base, 2, 1);
%     AA_w(i) = custom_main_snake(amplitude_base, frequency_base, p, phase_offset_base, 2, 2);
% end
% 
% % PHASE
% LL_ph = zeros(size(number_vals));
% LA_ph = zeros(size(number_vals));
% AL_ph = zeros(size(number_vals));
% AA_ph = zeros(size(number_vals));
% 
% parameter = phase_offset_vals;
% 
% for i = 1:length(phase_offset_vals)
% 
%     p = parameter(i);
% 
%     LL_ph(i) = custom_main_snake(amplitude_base, frequency_base, wavelength_base, p, 1, 1);
%     LA_ph(i) = custom_main_snake(amplitude_base, frequency_base, wavelength_base, p, 1, 2);
%     AL_ph(i) = custom_main_snake(amplitude_base, frequency_base, wavelength_base, p, 2, 1);
%     AA_ph(i) = custom_main_snake(amplitude_base, frequency_base, wavelength_base, p, 2, 2);
% end


% 
% % --- Plot ---
% figure;
% plot(amp_norm, dist_amplitude, 'r', 'LineWidth', 2);
% hold on;
% plot(freq_norm, dist_frequency, 'g', 'LineWidth', 2);
% plot(wave_norm, dist_wavelength, 'b', 'LineWidth', 2);
% plot(phase_norm, dist_phase_offset, 'y', 'LineWidth', 2);
% 
% 
% xlabel('Parameter Value');
% ylabel('Distance Traveled');
% legend('Amplitude Sweep', 'Frequency Sweep', 'Wavelength Sweep', 'Phase Sweep');
% grid on;
%

% % % AMPLITUDE
% figure;
% plot(amplitude_vals, LL_amp, '.', 'LineWidth',2, 'MarkerSize', 15); hold on;
% plot(amplitude_vals, LA_amp, '.', 'LineWidth',2, 'MarkerSize', 15);
% plot(amplitude_vals, AL_amp, '.', 'LineWidth',2, 'MarkerSize', 15);
% plot(amplitude_vals, AA_amp, '.', 'LineWidth',2, 'MarkerSize', 15);
% 
% legend('Land Loco + Land Env', ...
%        'Land Loco + Aquatic Env', ...
%        'Aquatic Loco + Land Env', ...
%        'Aquatic Loco + Aquatic Env');
% 
% xlabel('Amplitude');
% ylabel('Distance Traveled');
% grid on;

% % 
% % % FREQUENCY
% figure;
% plot(frequency_vals, LL_freq, '.', 'LineWidth',2, 'MarkerSize', 15); hold on;
% plot(frequency_vals, LA_freq, '.', 'LineWidth',2, 'MarkerSize', 15);
% plot(frequency_vals, AL_freq, '.', 'LineWidth',2, 'MarkerSize', 15);
% plot(frequency_vals, AA_freq, '.', 'LineWidth',2, 'MarkerSize', 15);
% 
% legend('Land Loco + Land Env', ...
%        'Land Loco + Aquatic Env', ...
%        'Aquatic Loco + Land Env', ...
%        'Aquatic Loco + Aquatic Env');
% 
% xlabel('Frequency');
% ylabel('Distance Traveled');
% grid on
% 
% % WAVELENGTH
% figure;
% plot(wavelength_vals, LL_w, '.', 'LineWidth',2, 'MarkerSize', 15); hold on;
% plot(wavelength_vals, LA_w, '.', 'LineWidth',2, 'MarkerSize', 15);
% plot(wavelength_vals, AL_w, '.', 'LineWidth',2, 'MarkerSize', 15);
% plot(wavelength_vals, AA_w, '.', 'LineWidth',2, 'MarkerSize', 15);
% 
% legend('Land Loco + Land Env', ...
%        'Land Loco + Aquatic Env', ...
%        'Aquatic Loco + Land Env', ...
%        'Aquatic Loco + Aquatic Env');
% 
% xlabel('Wavelength');
% ylabel('Distance Traveled');
% grid on

% % % % PHASE
% figure;
% plot(phase_offset_vals, LL_ph, '.', 'LineWidth',2, 'MarkerSize', 15); hold on;
% plot(phase_offset_vals, LA_ph, '.', 'LineWidth',2, 'MarkerSize', 15);
% plot(phase_offset_vals, AL_ph, '.', 'LineWidth',2, 'MarkerSize', 15);
% plot(phase_offset_vals, AA_ph, '.', 'LineWidth',2, 'MarkerSize', 15);
% 
% legend('Land Loco + Land Env', ...
%        'Land Loco + Aquatic Env', ...
%        'Aquatic Loco + Land Env', ...
%        'Aquatic Loco + Aquatic Env');
% 
% xlabel('Phase Offset');
% ylabel('Distance Traveled');
% grid on