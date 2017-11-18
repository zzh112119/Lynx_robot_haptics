% this function is similar to computeSurfaceRepel but adds distance check
% to simulate different feedback at different location

function F_btn=computeBtnForce(surface_b,c_surface,pos0)
global posEE;
global BtnFlag;

F_btn= [0;0;0];

X1 = surface_b(:, 2) - surface_b(:, 1);
X2 = surface_b(:, 4) - surface_b(:, 1);
X3 = surface_b(:, 2) - surface_b(:, 3);
X4 = surface_b(:, 4) - surface_b(:, 3);
x1 = posEE' - surface_b(:, 1);
x4 = posEE' - surface_b(:, 3);

if sum(x1 .* X1) >= 0 & sum(x1 .* X2) >= 0 & sum(x4 .* X3) >= 0 & sum(x4 .* X4) >= 0
    ns = [(X2(3)*X1(2)-X1(3)*X2(2))/(X1(1)*X2(2) - X2(1)*X1(2)); 0; 1];
    ns(2) = (-X1(3) - X1(1) * ns(1)) / X1(2);
    if isnan(ns(2))
    ns = [1;0;0];
    end
    if sum(ns .* (pos0 - surface_b(:, 1))) == 0
        warning('Initial position on plane: May not be able to define plane direction')
    end
    if sum(ns .* (pos0 - surface_b(:, 1))) < 0
        ns = -ns;
    end
    if sum(ns .* x1) < 0
        if norm((x1 .* ns/norm(ns))) < 30 %if the position is less than 30, calculates force proportional to the position
            F_btn = 0.2* norm((x1 .* ns/norm(ns))) * (ns / norm(ns))/10;
        end
        if norm((x1 .* ns/norm(ns)))>30 && norm((x1 .* ns/norm(ns)))<40 % if position is in the 'blank' area, force equals to zero
            F_btn = 0;
        end
        if norm((x1 .* ns/norm(ns))) > 40 %if larger than 40, use wall configuration
            F_btn = c_surface * (norm((x1 .* ns/norm(ns))) -40) .* (ns / norm(ns));
            if BtnFlag == 1
                BtnFlag = 0;
            else
                BtnFlag = 1;
            end
        end
    end
end

end