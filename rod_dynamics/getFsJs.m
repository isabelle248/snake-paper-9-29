function [Fs, Js, stretch_springs] = getFsJs(MultiRod, stretch_springs, q)
% Same result as the original; the spring force and Jacobian are now computed
% on the 6 local DOFs only (not on n_DOF x n_DOF arrays), which is much faster.
n_stretch = numel(stretch_springs);
n_DOF = MultiRod.n_DOF;

Fs = zeros(n_DOF,1);
Js = zeros(n_DOF);

for c = 1:n_stretch
    n0=stretch_springs(c).nodes_ind(1);
    n1=stretch_springs(c).nodes_ind(2);
    node0p = q(mapNodetoDOF(n0))';
    node1p = q(mapNodetoDOF(n1))';
    ind = stretch_springs(c).ind;
    [dF, dJ] = gradEs_hessEs_struct(6, (1:6)', node0p, node1p, stretch_springs(c));

    Fs(ind) = Fs(ind) - dF;
    Js(ind, ind) = Js(ind, ind) - dJ;

    stretch_springs(c).dF = dF;
    stretch_springs(c).dJ = dJ;
end
end
