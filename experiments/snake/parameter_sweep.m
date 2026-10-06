% parameter_sweep.m
% One-parameter-at-a-time sweeps for all four gait / environment cases.
% Run this script from the MAT-DiSMech root folder.
%
% Changes from the old version (see the bug-fix plan):
%  - all four cases run in one loop (no commented-out blocks)
%  - 20 points, never linspace(a,b,1) (that returns b only)
%  - baseline = body shape of a real snake, not the arbitrary "best amplitude"
%  - each sweep ends where the model stops being valid (snake_gait_check.m)
%  - one failed run does not stop the sweep (try/catch, NaN)
%  - results go to a CSV file and a .mat file

clc; close all;
addpath springs/ util_functions/ contact_functions/ rod_dynamics/ shell_dynamics/ external_forces/ adaptive_stepping/ logging/
addpath(genpath('experiments'));

% ================================================================
% CHOOSE PARAMETER TO SWEEP: "amplitude", "frequency" or "wavelength"
% ================================================================
sweep_parameter = "amplitude";

% Baseline: the body shape of a real snake (lateral amplitude 0.10 body lengths,
% wavelength along the direction of travel 0.42 body lengths; Hu et al. 2009).
amplitude_base  = 0.6;     % joint angle 2*atan(0.3) = 33 deg
wavelength_base = 0.6;     % body lengths
frequency_base  = 1.825;   % Hz
phase_base      = pi/2;    % a constant phase only shifts the time origin (no effect)

opts = struct();           % defaults: 10 s measured after a 2-cycle ramp, quadratic water drag
n_points = 20;             % never 1

% Sweep ranges. Upper ends of amplitude and wavelength are set by the validity rules
% (45 deg joint angle, 8 joints per wavelength, no self-contact), for both gaits.
robotDescriptionSnake;     % only to get nodes and geom.rod_r0
A_hi = valid_sweep_limit("amplitude",  0.2, 2*tan(pi/8), amplitude_base, wavelength_base, nodes, geom.rod_r0);
L_hi = valid_sweep_limit("wavelength", 0.4, 2.0,   amplitude_base, wavelength_base, nodes, geom.rod_r0);
amplitude_vals  = linspace(0.2, A_hi, n_points);   % below 0.2 the land result depends on the friction smoothing velTol
wavelength_vals = linspace(0.4, L_hi, n_points);
frequency_vals  = linspace(1.0, 2.65, n_points);
fprintf('amplitude range %.3f to %.3f, wavelength range %.3f to %.3f\n', 0.2, A_hi, 0.4, L_hi);

switch sweep_parameter
    case "amplitude",  parameter_vals = amplitude_vals;  parameter_name = 'Amplitude (joint bend 2tan(\phi/2))';
    case "frequency",  parameter_vals = frequency_vals;  parameter_name = 'Frequency (Hz)';
    case "wavelength", parameter_vals = wavelength_vals; parameter_name = 'Wavelength (body lengths)';
    otherwise, error('Invalid sweep parameter.');
end

cases = [1 1; 1 2; 2 1; 2 2];          % [loco_type env_type]
case_names = {'Land Loco + Land Env', 'Land Loco + Aquatic Env', 'Aquatic Loco + Land Env', 'Aquatic Loco + Aquatic Env'};
n = numel(parameter_vals);
R = nan(4*n, 14);                       % one row per simulation
row = 0;
for i = 1:n
    p = parameter_vals(i);
    amp = amplitude_base; freq = frequency_base; wavelen = wavelength_base;
    switch sweep_parameter
        case "amplitude",  amp = p;
        case "frequency",  freq = p;
        case "wavelength", wavelen = p;
    end
    for c = 1:4
        row = row + 1;
        R(row, 1:3) = [p, cases(c,1), cases(c,2)];
        try
            [fwd, tot, lat, yaw, spd, cyc, info] = custom_main_snake(amp, freq, wavelen, phase_base, cases(c,1), cases(c,2), opts);
            R(row, 4:9) = [fwd, tot, lat, yaw, spd, cyc];
            R(row, 12) = info.gait.is_valid;
            R(row, 13) = info.simulated;
            if info.simulated
                R(row, 10) = info.travel_angle_deg;
                R(row, 11) = info.min_gap;
                R(row, 14) = info.self_contact;   % 1 = simulated body touched itself (result set to NaN)
            end
        catch err
            fprintf('Run failed (p = %g, case %d): %s\n', p, c, err.message);
        end
        fprintf('%s = %.4f  %-28s  forward = %.5f m\n', sweep_parameter, p, case_names{c}, R(row,4));
    end
end

% ================================================================
% SAVE
% ================================================================
header = {'SweepValue','LocoType','EnvType','ForwardDisp','TotalDisp','LateralDrift','NetYaw', ...
          'MeanSpeedSteady','DistancePerCycle','TravelAngleDeg','MinGap','Valid','Simulated','SelfContact'};
fname = sprintf('%s_metrics.csv', sweep_parameter);
fid = fopen(fname, 'w');
fprintf(fid, '%s,', header{1:end-1}); fprintf(fid, '%s\n', header{end});
fprintf(fid, [repmat('%.10g,', 1, 13) '%.10g\n'], R');
fclose(fid);
save(sprintf('%s_metrics.mat', sweep_parameter), 'R', 'header', 'parameter_vals', 'amplitude_base', ...
     'wavelength_base', 'frequency_base', 'opts');

% ================================================================
% PLOT SIGNED FORWARD DISPLACEMENT (points outside the valid range are not drawn)
% ================================================================
figure; hold on;
for c = 1:4
    sel = R(:,2) == cases(c,1) & R(:,3) == cases(c,2);
    plot(R(sel,1), R(sel,4), '.-', 'DisplayName', case_names{c}, 'LineWidth', 1.5, 'MarkerSize', 15);
end
yline(0, '--', 'HandleVisibility', 'off');
xlabel(parameter_name); ylabel('Signed forward displacement in 10 s (m)');
title(['Forward displacement vs ' char(sweep_parameter)]);
legend('Location', 'best'); grid on; hold off;
