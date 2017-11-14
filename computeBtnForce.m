function F_btn=computeBtnForce(surface_b,c_surface,pos0)
global posEE;
global BtnFlag;

F_btn= [0;0;0];

X1 = surface_b(:, 2) - surface_b(:, 1);
X2 = surface_b(:, 3) - surface_b(:, 1);
X3 = surface_b(:, 2) - surface_b(:, 4);
X4 = surface_b(:, 3) - surface_b(:, 4);
x1 = posEE - surface_b(:, 1);
x4 = posEE - surface_b(:, 4);
if sum(x1 .* X1) >= 0 & sum(x1 .* X2) >= 0 & sum(x4 .* X3) >= 0 & sum(x4 .* X4) >= 0
    ns = [(X2(3)*X1(2)-X1(3)*X2(2))/(X1(1)*X2(2) - X2(1)*X1(2)); 0; 1];
    ns(2) = (-X1(3) - X1(1) * ns(1)) / X1(2);
    if sum(ns .* (pos0 - surface_b(:, 1))) == 0
        warning('Initial position on plane: May not be able to define plane direction')
    end
    if sum(ns .* (pos0 - surface_b(:, 1))) < 0
        ns = -ns;
    end
    if sum(ns .* x1) < 0
        if norm(ns .* x1) < 30
            F_btn = norm(ns .* x1) .* sqrt(sum((x1 .* ns/norm(ns)) .^ 2)) * (ns / norm(ns));
        end
        if norm(ns .* x1)>30 && norm(ns .* x1)<35
            F_btn=0;
        end
        if norm(ns .* x1) > 35
            F_btn = c_surface * sqrt(sum((x1 .* ns/norm(ns)) .^ 2)) * (ns / norm(ns));
            if BtnFlag == 1
                BtnFlag=0;
            else
                BtnFlag=1;
            end
        end
    end
end

end