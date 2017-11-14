function F = computeTextureForce(Fnq, character, velocity)
global posEE;

Fk = norm(Fnq);
F = double(subs(character, {'Fn', 'v', 'pos'}, {Fk, velocity, posEE}));