% joint_sweep.m  -- joint sweep for reviewer comment MF9
% Run this script from the MAT-DiSMech root folder.
%
% Why the joint sweep is 2-D (amplitude x wavelength) and not 3-D or 4-D:
%   * a constant phase only shifts the time origin, so it has no effect on the motion;
%   * frequency only rescales time: the distance per cycle changes by less than 4 %
%     between 1 and 2.65 Hz (checked with quadratic water drag; NOT valid for the
%     "stokes" water model), so displacement in 10 s = 10 * f * (distance per cycle).
% So the response surface is displacement(A, lambda) at one frequency. Frequency is
% checked by confirmation runs at the predicted optimum at 1 Hz and 2.65 Hz.
%
% To use several CPU cores without the Parallel Computing Toolbox, open up to 4 MATLAB
% windows and set a different case_id in each one. With the toolbox, change the inner
% "for j" loop of the coarse grid to "parfor j".

clc; close all;
addpath springs/ util_functions/ contact_functions/ rod_dynamics/ shell_dynamics/ external_forces/ adaptive_stepping/ logging/
addpath(genpath('experiments'));

case_id = 1;                      % 1 = land gait/land, 2 = land gait/water, 3 = aquatic gait/land, 4 = aquatic gait/water
cases = [1 1; 1 2; 2 1; 2 2];     % [loco_type env_type]
case_names = {'Land Loco + Land Env', 'Land Loco + Aquatic Env', 'Aquatic Loco + Land Env', 'Aquatic Loco + Aquatic Env'};
loco = cases(case_id, 1); envt = cases(case_id, 2);

if envt == 1
    A_lo = 0.2;                       % below 0.2 the land result depends on the friction smoothing velTol
else
    A_lo = 0.1;
end
A_grid = linspace(A_lo, 2*tan(pi/8), 15);   % upper end = 45 deg joint angle
L_grid = linspace(0.4, 2.0, 15);            % body lengths
freq   = 1.825;                             % Hz
phase  = pi/2;
opts   = struct();                          % 10 s measured after a 2-cycle ramp
refine = true;                              % finer 5 x 5 grid around the best coarse point

robotDescriptionSnake;                % to get nodes and geom.rod_r0
nA = numel(A_grid); nL = numel(L_grid);
D = nan(nA, nL);                      % forward displacement in 10 s (m)
TRAV = nan(nA, nL);                   % travel direction (deg)
valid = false(nA, nL);                % pre-check (snake_gait_check); runs where the simulated body
                                      % touched itself also come back as NaN
for i = 1:nA
    for j = 1:nL
        g = snake_gait_check(A_grid(i), L_grid(j), loco, nodes, geom.rod_r0);
        valid(i,j) = g.is_valid;
    end
end
fprintf('%s: %d valid grid points of %d\n', case_names{case_id}, nnz(valid), numel(valid));

t_start = tic;
for i = 1:nA
    for j = 1:nL
        if ~valid(i,j), continue; end
        try
            [fwd, ~, ~, ~, ~, ~, info] = custom_main_snake(A_grid(i), freq, L_grid(j), phase, loco, envt, opts);
            D(i,j) = fwd;
            TRAV(i,j) = info.travel_angle_deg;
        catch err
            fprintf('Run failed (A = %g, lambda = %g): %s\n', A_grid(i), L_grid(j), err.message);
        end
    end
    fprintf('row %d of %d done, %.1f min so far\n', i, nA, toc(t_start)/60);
end
save(sprintf('joint_sweep_case%d.mat', case_id), 'A_grid', 'L_grid', 'D', 'TRAV', 'valid', 'freq', 'opts', 'loco', 'envt');

% ---------------- best coarse point ----------------
[best, idx] = max(D(:));
[ib, jb] = ind2sub(size(D), idx);
A_opt = A_grid(ib); L_opt = L_grid(jb);
fprintf('Best coarse grid point: A = %.4f, lambda = %.4f, %.4f m in 10 s at %.3f Hz\n', A_opt, L_opt, best, freq);

% ---------------- optional local refinement (5 x 5, half the grid spacing) ----------------
A_ref = []; L_ref = []; D_ref = [];
if refine
    dA = A_grid(2) - A_grid(1); dL = L_grid(2) - L_grid(1);
    A_ref = A_opt + dA*(-1:0.5:1);
    L_ref = L_opt + dL*(-1:0.5:1);
    D_ref = nan(5, 5);
    for i = 1:5
        for j = 1:5
            a = A_ref(i); w = L_ref(j);
            if a < A_grid(1) - 1e-12 || a > A_grid(end) + 1e-12 || w < L_grid(1) - 1e-12 || w > L_grid(end) + 1e-12
                continue;                    % stay inside the tested range
            end
            g = snake_gait_check(a, w, loco, nodes, geom.rod_r0);
            if ~g.is_valid, continue; end
            try
                D_ref(i,j) = custom_main_snake(a, freq, w, phase, loco, envt, opts);
            catch err
                fprintf('Refinement run failed (A = %g, lambda = %g): %s\n', a, w, err.message);
            end
        end
    end
    [best_ref, idx] = max(D_ref(:));
    if best_ref > best
        [ir, jr] = ind2sub(size(D_ref), idx);
        best = best_ref; A_opt = A_ref(ir); L_opt = L_ref(jr);
    end
    fprintf('Optimum after refinement: A = %.4f, lambda = %.4f, %.4f m in 10 s\n', A_opt, L_opt, best);
end

% Is the optimum next to an invalid point or at the edge of the tested range?
on_edge = A_opt <= A_grid(1) + 1e-12 || A_opt >= A_grid(end) - 1e-12 || ...
          L_opt <= L_grid(1) + 1e-12 || L_opt >= L_grid(end) - 1e-12;
[~, ib] = min(abs(A_grid - A_opt)); [~, jb] = min(abs(L_grid - L_opt));
for di = -1:1
    for dj = -1:1
        ii = ib + di; jj = jb + dj;
        if ii >= 1 && ii <= nA && jj >= 1 && jj <= nL && (~valid(ii,jj) || isnan(D(ii,jj)))
            on_edge = true;
        end
    end
end
if on_edge
    fprintf('The optimum lies at the edge of the tested or valid range.\n');
end
save(sprintf('joint_sweep_case%d.mat', case_id), 'A_ref', 'L_ref', 'D_ref', 'A_opt', 'L_opt', 'best', 'on_edge', '-append');

% ---------------- confirmation runs at other frequencies ----------------
f_check = [1.0, 2.65];
confirm = nan(numel(f_check), 3);     % [frequency, predicted, simulated]
for k = 1:numel(f_check)
    confirm(k,1:2) = [f_check(k), best*f_check(k)/freq];
    try
        confirm(k,3) = custom_main_snake(A_opt, f_check(k), L_opt, phase, loco, envt, opts);
    catch err
        fprintf('Confirmation run failed (f = %g): %s\n', f_check(k), err.message);
    end
    fprintf('Confirmation at %.3f Hz: predicted %.4f m, simulated %.4f m (%.1f %%)\n', ...
        confirm(k,1), confirm(k,2), confirm(k,3), 100*(confirm(k,3) - confirm(k,2))/confirm(k,2));
end
save(sprintf('joint_sweep_case%d.mat', case_id), 'confirm', '-append');

% ---------------- response surface ----------------
figure;
h = imagesc(L_grid, A_grid, D);
set(h, 'AlphaData', ~isnan(D));       % invalid points are left blank
set(gca, 'YDir', 'normal'); colorbar; hold on;
plot(L_opt, A_opt, 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'w');
xlabel('Wavelength (body lengths)'); ylabel('Amplitude (joint bend)');
title(sprintf('%s: forward displacement in 10 s (m), f = %.3f Hz', case_names{case_id}, freq));
hold off;
