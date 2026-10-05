function [Ft, Jt, bend_twist_springs] = getFtJt(MultiRod, bend_twist_springs, q, refTwist, sim_params)
% Same result as the original; the spring force and Jacobian are now computed
% on the 11 local DOFs only (not on n_DOF x n_DOF arrays), which is much faster.
n_DOF = MultiRod.n_DOF;
n_nodes = MultiRod.n_nodes;
n_twist = numel(bend_twist_springs);
undef_refTwist = MultiRod.undef_refTwist;
loc = (1:11)';

Ft = zeros(n_DOF,1);
Jt = zeros(n_DOF);

for c = 1:n_twist
    n0 = bend_twist_springs(c).nodes_ind(1);
    n1 = bend_twist_springs(c).nodes_ind(2);
    n2 = bend_twist_springs(c).nodes_ind(3);
    e0 = bend_twist_springs(c).edges_ind(1);
    e1 = bend_twist_springs(c).edges_ind(2);

    node0p = q(mapNodetoDOF(n0))';
    node1p = q(mapNodetoDOF(n1))';
    node2p = q(mapNodetoDOF(n2))';

    theta_e = bend_twist_springs(c).sgn(1) * q(mapEdgetoDOF(e0, n_nodes));
    theta_f = bend_twist_springs(c).sgn(2) * q(mapEdgetoDOF(e1, n_nodes));

    ind = bend_twist_springs(c).ind; % 11 global DOF indices

    if(isfield(sim_params, 'bergou_DER') && sim_params.bergou_DER)
        [dF, dJ] = gradEt_hessEt_struct_new(11, loc, node0p, node1p, node2p, ...
            theta_e, theta_f, refTwist(c), bend_twist_springs(c), undef_refTwist(c));
    else
        [dF, dJ] = gradEt_hessEt_panetta(11, loc, node0p, node1p, node2p, ...
            theta_e, theta_f, refTwist(c), bend_twist_springs(c), undef_refTwist(c));
    end

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

    Ft(ind) = Ft(ind) - dF;
    Jt(ind, ind) = Jt(ind, ind) - dJ;

    bend_twist_springs(c).dFt = dF;
    bend_twist_springs(c).dJt = dJ;
end
end
