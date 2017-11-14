%q is the six joint angles, x is the position of goal, Spotential is the
%attractive field strength

function Fa = computeAttForce(Pg,Spotential)

global posEE

if norm(Pg-posEE) <= 100
    Fa = Spotential * (Pg-posEE) / norm(Pg-posEE); 
end

end