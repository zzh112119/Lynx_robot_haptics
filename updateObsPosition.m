function obst_dull = updateObsPosition(obst, Pg, Fk)
global deltaT

Spotential = 0.5;
Fa = [0;0;0];
if (norm(Pg-obst.pos) > 0)
    Fa = Spotential * (Pg-obst.pos) / norm(Pg-obst.pos); 
end

accelerate =  - (Fk - Fa) / obst.mass;

obst_dull.mass = obst.mass;
obst_dull.r = obst.r;
obst_dull.pos = obst.pos + deltaT * obst.v + (1 / 2) * deltaT ^ 2 * accelerate;
obst_dull.v = obst.v + deltaT * accelerate;
end