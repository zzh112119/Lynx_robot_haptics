function Fp = computeSurfaceRepel(posEE, surface, pos0)
c_surface = 100;
Fp = [0;0;0];

X1 = surface(:, 2) - surface(:, 1);
X2 = surface(:, 3) - surface(:, 1);
X3 = surface(:, 2) - surface(:, 4);
X4 = surface(:, 3) - surface(:, 4);
x1 = posEE - surface(:, 1);
x4 = posEE - surface(:, 4);
if sum(x1 .* X1) >= 0 & sum(x1 .* X2) >= 0 & sum(x4 .* X3) >= 0 & sum(x4 .* X4) >= 0
    ns = [(X2(3)*X1(2)-X1(3)*X2(2))/(X1(1)*X2(2) - X2(1)*X1(2)); 0; 1];
    ns(2) = (-X1(3) - X1(1) * ns(1)) / X1(2);
    if sum(ns .* (pos0 - surface(:, 1))) == 0
        warning('Initial position on plane: May not be able to define plane direction')
    end
    if sum(ns .* (pos0 - surface(:, 1))) < 0
        ns = -ns;
    end
    if sum(ns .* x1) < 0
        Fp = c_surface * sqrt(sum((x1 .* ns) .^ 2)) * (ns / norm(ns));
    end
end
end