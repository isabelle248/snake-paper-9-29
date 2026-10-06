function gmin = body_min_gap(P, min_sep)
% Smallest distance between two segments of the centre line that are min_sep or
% more segments apart. P: n x 3 node positions (one row per node).
n_seg = size(P, 1) - 1;
gmin = inf;
for i = 1:n_seg
    for j = i + min_sep:n_seg
        gmin = min(gmin, seg_seg_dist(P(i,:), P(i+1,:), P(j,:), P(j+1,:)));
    end
end
end

function d = seg_seg_dist(p1, q1, p2, q2)
% Closest distance between segments p1-q1 and p2-q2 (Ericson, Real-Time Collision Detection, 2005)
d1 = q1 - p1; d2 = q2 - p2; r = p1 - p2;
a = d1*d1'; e = d2*d2'; f = d2*r'; c = d1*r'; b = d1*d2';
den = a*e - b*b;
if den > 1e-20
    s = min(max((b*f - c*e)/den, 0), 1);
else
    s = 0;
end
t = (b*s + f)/e;
if t < 0
    t = 0; s = min(max(-c/a, 0), 1);
elseif t > 1
    t = 1; s = min(max((b - c)/a, 0), 1);
end
d = norm((p1 + d1*s) - (p2 + d2*t));
end
