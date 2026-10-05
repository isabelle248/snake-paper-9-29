function [Fb, Jb, bend_twist_springs] = getFbJb(MultiRod, bend_twist_springs, q, m1, m2, sim_params)
% Same result as the original; the spring force and Jacobian are now computed
% on the 11 local DOFs only (not on n_DOF x n_DOF arrays), which is much faster.
n_DOF = MultiRod.n_DOF;
n_bend = numel(bend_twist_springs);
loc = (1:11)';

Fb = zeros(n_DOF,1);
Jb = zeros(n_DOF);

for c = 1:n_bend
    n0 = bend_twist_springs(c).nodes_ind(1);
    n1 = bend_twist_springs(c).nodes_ind(2);
    n2 = bend_twist_springs(c).nodes_ind(3);
    e0 = bend_twist_springs(c).edges_ind(1);
    e1 = bend_twist_springs(c).edges_ind(2);

    node0p = q(mapNodetoDOF(n0))';
    node1p = q(mapNodetoDOF(n1))';
    node2p = q(mapNodetoDOF(n2))';

    m1e = m1(e0,:);
    m2e = bend_twist_springs(c).sgn(1) * m2(e0,:);
    m1f = m1(e1,:);
    m2f = bend_twist_springs(c).sgn(2) * m2(e1,:);

    ind = bend_twist_springs(c).ind; % 11 global DOF indices

    if(isfield(sim_params, 'bergou_DER') && sim_params.bergou_DER)
        [dF, dJ] = gradEb_hessEb_struct(11, loc, node0p, node1p, node2p, m1e, m2e, m1f, m2f, bend_twist_springs(c));
    else
        [dF, dJ] = gradEb_hessEb_panetta(11, loc, node0p, node1p, node2p, m1e, m2e, m1f, m2f, bend_twist_springs(c));
    end

    % change sign of forces if the edges were flipped for alignment earlier
    % (local DOF 10 is edge e0, local DOF 11 is edge e1)
    if bend_twist_springs(c).sgn(1) ~= 1
        dF(10) = - dF(10);
        dJ(10, :) = - dJ(10, :);
        dJ(:, 10) = - dJ(:, 10);
    end
    if bend_twist_springs(c).sgn(2) ~= 1
        dF(11) = - dF(11);
        dJ(11, :) = - dJ(11, :);
        dJ(:, 11) = - dJ(:, 11);
    end

    Fb(ind) = Fb(ind) - dF;
    Jb(ind, ind) = Jb(ind, ind) - dJ;

    bend_twist_springs(c).dFb = dF;
    bend_twist_springs(c).dJb = dJ;
end
end
