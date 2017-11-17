function obst_dull = updateObsPosition(obst, Pg, Fk, surfs)
global deltaT

Spotential = 0.5;
epsilon = 1;
Fa = [0;0;0];
if (norm(Pg-obst.pos) > 0)
    Fa = Spotential * (Pg-obst.pos) / norm(Pg-obst.pos); 
end

accelerate = 0;% - (Fk - Fa) / obst.mass;

obst_dull = struct('mass', 1, 'r', 0, 'pos', [0;0;0], 'v', [0;0;0]);

obst_dull.mass = obst.mass;
obst_dull.r = obst.r;
obst_dull.pos = obst.pos + deltaT * obst.v + (1 / 2) * deltaT ^ 2 * accelerate;
obst_dull.v = obst.v + deltaT * accelerate;

for i = 1 : length(surfs)
    [dis, ns] = psdist3(obst_dull.pos, surfs{i});
    ns = ns / norm(ns);
    if dis < obst_dull.r
        obst_dull.pos = obst.pos;
        obst_dull.pos'
        obst_dull.v = -epsilon * sum(obst.v .* ns) * ns + (obst.v - sum(obst.v .* ns) * ns) + deltaT * accelerate;
        obst_dull.v'
    end
end

end