function [F_Rep] = computeRepForce(q, obs, ita)
%Calculate 
[X, ~] = updateQ(q);

Xo = obs(1);
Yo = obs(2);
Zo = obs(3);

rho = -35 * ones(1, 5);
dir = zeros(5, 3);
rho_o = 1e-10 + [0 20-1e-10 20-1e-10 30-1e-10 20-1e-10];
Uq = zeros(1, 5);
for i = 1 : 5
    x1 = X(i, 1);
    x2 = X(i + 1, 1);
    y1 = X(i, 2);
    y2 = X(i + 1, 2);
    z1 = X(i, 3);
    z2 = X(i + 1, 3);
    
    [tmp, dir(i, :)] = pldist3(x1, y1, z1, x2, y2, z2, Xo, Yo, Zo);
    rho(i) = rho(i) + tmp;
    Uq(i) = (1 / rho(i) - 1 / rho_o(i)) ^ 2;
    if rho(i) > 1.25 * rho_o(i)
        Uq(i) = 0;
        F_Rep(i, :) = [0 0 0];
    else
        if rho(i) > 0
            F_RepMag(i) = ita * (1 / rho(i) - 1 / rho_o(i)) * (1 / (rho(i))) ^ 2;
            F_Rep(i, :) = F_RepMag(i) * dir(i, :);
        else
            F_Rep(i, :) = Inf * dir(i, :);
        end
    end
end
F_Rep(6, :) = F_Rep(5, :);
F_Rep(5, :) = F_Rep(4, :);
% [val, idx] = max(Uq);
% if val == 0
%     F_Rep = [0 0 0];
%     return
% end
% 
% F_Rep = ita * (1 / rho(idx) - 1 / rho_o(idx)) * (1 / (rho(idx))) ^ 2;
% F_Rep = F_Rep * dir(idx, :);
end