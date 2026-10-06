function c = crossFast(a, b)
% Cross product of two 3-vectors. Same result as cross(a,b), much faster.
% The output has the same orientation (row or column) as a.
c = [a(2)*b(3)-a(3)*b(2), a(3)*b(1)-a(1)*b(3), a(1)*b(2)-a(2)*b(1)];
if size(a,1) == 3
    c = c.';
end
end
