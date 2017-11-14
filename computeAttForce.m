%q is the six joint angles, x is the position of goal, Spotential is the
%attractive field strength

function Fa = computeAttForce(Pg,Spotential)

global posEE

Fa = Spotential*(Pg-posEE); 

end