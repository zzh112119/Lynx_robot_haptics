% this function updates the position of the obstacle based on position,
% velocity, acceleration and etc.

function obst_dull = updateObsPosition(obst, Pg, Fk, surfs)
global deltaT

Spotential = 0.5;
epsilon = 0.2;
Fa = [0;0;0];
if (norm(Pg-obst.pos) > 0)
    Fa = Spotential * (Pg-obst.pos) / norm(Pg-obst.pos); 
end

accelerate =  - (Fk - Fa) / obst.mass; % Use F=ma to calculate current acceleration

obst_dull = struct('mass', 1, 'r', 0, 'pos', [0;0;0], 'v', [0;0;0]);

obst_dull.mass = obst.mass;
obst_dull.r = obst.r;
obst_dull.pos = obst.pos + deltaT * obst.v + (1 / 2) * deltaT ^ 2 .* accelerate;
obst_dull.v = obst.v + deltaT .* accelerate;

for i = 1 : length(surfs)
    [dis, ns, Fflag] = psdist3(obst_dull.pos, surfs{i});
    ns = ns / norm(ns);
    if (dis < obst_dull.r && Fflag) % if ball collides with wall, repound the ball and give it a velocity opposite to original velocity
        obst_dull.pos = obst.pos;
        obst_dull.v = -epsilon * sum(obst.v .* ns) * ns + (obst.v - sum(obst.v .* ns) * ns) + deltaT * accelerate;
    end
end

end