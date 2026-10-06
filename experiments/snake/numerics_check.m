% numerics_check.m -- numerical checks for reviewer comment MF8 (and MF2)
% For each test point it runs:
%   base   : 21 nodes, 100 steps per cycle, Newton tolerances 1e-4 (the normal settings)
%   mesh   : 41 nodes with the same physical body shape
%   dt     : 200 steps per cycle (half the time step)
%   tol    : Newton tolerances 1e-6 (1e-7 does not converge on land in the first steps,
%            where the snake is almost at rest and the friction is smoothed)
% and prints the change of the forward displacement in percent.
% Run this script from the MAT-DiSMech root folder.
%
% Same physical shape on the 41-node body: each 21-node joint becomes two joints, so the
% joint angle is halved:  A41 = 2*tan(atan(A21/2)/2).

clc; close all;
addpath springs/ util_functions/ contact_functions/ rod_dynamics/ shell_dynamics/ external_forces/ adaptive_stepping/ logging/
addpath(genpath('experiments'));

% test points: [A21, wavelength, frequency, loco_type, env_type]
% the baseline, then the optima found by joint_sweep.m (replace with your own optima)
points = [0.6     0.6   1.825 1 1;
          0.6     0.6   1.825 2 2;
          0.3796  1.2   1.825 1 1;
          0.4244  1.2   1.825 2 1;
          0.2041  2.0   1.825 1 2;
          0.2301  2.0   1.825 2 2];

variants = {struct(), struct('node_file', 'experiments/snake/horizontal_rod_n41.txt'), ...
            struct('steps_per_cycle', 200), struct('tol', 1e-6)};
R = nan(size(points, 1), 4);
for k = 1:size(points, 1)
    p = points(k, :);
    for v = 1:4
        A = p(1);
        if v == 2, A = 2*tan(atan(p(1)/2)/2); end
        try
            R(k, v) = custom_main_snake(A, p(3), p(2), pi/2, p(4), p(5), variants{v});
        catch err
            fprintf('Run failed (point %d, variant %d): %s\n', k, v, err.message);
        end
    end
    fprintf('A=%.4f lambda=%.3f f=%.3f loco=%d env=%d: base %.4f m | mesh %+.1f %% | dt/2 %+.2f %% | tol %+.3f %%\n', ...
        p(1), p(2), p(3), p(4), p(5), R(k,1), 100*(R(k,2) - R(k,1))/R(k,1), 100*(R(k,3) - R(k,1))/R(k,1), 100*(R(k,4) - R(k,1))/R(k,1));
end
save('numerics_check.mat', 'points', 'R');
