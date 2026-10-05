function g = snake_gait_check(amp, wavelen, loco_type, nodes, r0)
% Checks, BEFORE a run, if a gait is inside the range where the model is valid.
% It uses the commanded body shape. The simulated body follows it closely, but near the
% self-contact limit it can bend a few percent more, so custom_main_snake.m checks the
% simulated body again after the run.
%
% Rules (all three must hold):
%   1) every joint angle <= 45 deg       (for sharper bends the 21-node result changes by more
%                                         than 8 % when the mesh is refined; for loco_type 1
%                                         this is amp <= 2*tan(pi/8) = 0.828)
%   2) at least 8 joints per wavelength  (wavelen >= 0.4 for 21 nodes)
%   3) no self-contact: two segments that are 3 or more segments apart never come closer
%      than one body diameter (2*r0). The model has no self-contact force, so a body that
%      touches itself would pass through itself.
%
% Output struct g: max_joint_angle_deg, joints_per_wavelength, min_gap (m),
%                  ok_angle, ok_resolution, ok_contact, is_valid

n_nodes = size(nodes, 1);
n_bends = n_nodes - 2;
l_seg = norm(nodes(2,:) - nodes(1,:));
s = (1:n_bends)/(n_bends + 1);           % same positions as actuate_snake.m
k = 2*pi/wavelen;
if loco_type == 1
    A_s = amp*ones(1, n_bends);
else
    A_s = amp*(1 + 0.25*s)/1.25;
end

g.max_joint_angle_deg = 2*atan(max(abs(A_s))/2)*180/pi;
g.joints_per_wavelength = wavelen*(n_bends + 1);
g.min_gap = inf;
for tau = linspace(0, 1, 101)             % one full cycle; a constant phase does not change the set of shapes
    kappa = A_s .* sin(2*pi*tau - k*s);
    phi = 2*atan(kappa/2);                % turning angle at each joint
    ang = [0, cumsum(phi)];               % direction of each segment
    P = [[0, cumsum(l_seg*cos(ang))]', [0, cumsum(l_seg*sin(ang))]', zeros(n_nodes, 1)];
    g.min_gap = min(g.min_gap, body_min_gap(P, 3));
end
g.ok_angle      = g.max_joint_angle_deg <= 45 + 1e-9;
g.ok_resolution = g.joints_per_wavelength >= 8 - 1e-9;
g.ok_contact    = g.min_gap >= 2*r0;
g.is_valid      = g.ok_angle && g.ok_resolution && g.ok_contact;
end
