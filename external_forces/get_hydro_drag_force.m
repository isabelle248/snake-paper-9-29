function [F_full, J_full] = get_hydro_drag_force(softRobot, q, q0, env, sim_params)
% Quadratic (high Reynolds number) water drag on each node, resistive model of
% Taylor (1952):
%     f_i = -0.5*rho*d*l_i*( Cn*|v_n|*v_n + pi*Ct*|v_t|*v_t )
%   v_t : node velocity along the body (tangent t), v_n : velocity across the body
%   d   : body diameter, l_i : length of body that belongs to node i (Voronoi length)
%   Cn  : drag coefficient across the body, Ct : skin-friction coefficient along it
% The velocity is v = (q - q0)/dt (implicit), so the force is part of the Newton solve.
% The Jacobian ignores the change of the tangent with q (as get_rft_force.m does).

N  = softRobot.n_nodes;
dt = sim_params.dt;
ndof = numel(q);
F_full = zeros(ndof, 1);
J_full = zeros(ndof, ndof);

X = reshape(q(1:3*N), 3, N);
V = reshape(q(1:3*N) - q0(1:3*N), 3, N)/dt;
lv = softRobot.voronoiRefLen;
kn0 = 0.5*env.rho*env.d*env.Cn;
kt0 = 0.5*env.rho*env.d*pi*env.Ct;
I3 = eye(3);

for i = 1:N
    if i == 1
        t = X(:,2) - X(:,1);
    elseif i == N
        t = X(:,N) - X(:,N-1);
    else
        t = X(:,i+1) - X(:,i-1);
    end
    t = t/norm(t);
    v  = V(:,i);
    vt = (t'*v)*t;
    vn = v - vt;
    a = norm(vn); b = norm(vt);
    kn = kn0*lv(i); kt = kt0*lv(i);
    idx = 3*i-2:3*i;
    F_full(idx) = -kn*a*vn - kt*b*vt;
    Pt = t*t'; Pn = I3 - Pt;
    Jn = zeros(3); Jt = zeros(3);
    if a > 0, Jn = (a*I3 + (vn*vn')/a)*Pn; end
    if b > 0, Jt = (b*I3 + (vt*vt')/b)*Pt; end
    J_full(idx, idx) = -(kn*Jn + kt*Jt)/dt;
end
end
