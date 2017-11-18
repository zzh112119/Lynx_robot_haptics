function [Fp, ns, Fflag] = psdist3(pt, surface)

Fp = 0;
pos0 = [250;0;200];

X1 = surface(:, 2) - surface(:, 1);
X2 = surface(:, 4) - surface(:, 1);
X3 = surface(:, 2) - surface(:, 3);
X4 = surface(:, 4) - surface(:, 3);
x1 = pt - surface(:, 1);
x4 = pt - surface(:, 3);
ns = [0;0;0];

ns = [(X2(3)*X1(2)-X1(3)*X2(2))/(X1(1)*X2(2) - X2(1)*X1(2)); 0; 1];
ns(2) = (-X1(3) - X1(1) * ns(1)) / X1(2);
if isnan(ns(2))
    ns = [1;0;0];
end
if sum(ns .* (pos0 - surface(:, 1))) == 0
    warning('Initial position on plane: May not be able to define plane direction')
end
if sum(ns .* (pos0 - surface(:, 1))) < 0
    ns = -ns;
end
Fp = sqrt(sum((x1 .* ns ./ norm(ns)) .^ 2)) ;
Fflag = 0;

if sum(x1 .* X1) >= 0 & sum(x1 .* X2) >= 0 & sum(x4 .* X3) >= 0 & sum(x4 .* X4) >= 0
    Fflag = 1;
end
