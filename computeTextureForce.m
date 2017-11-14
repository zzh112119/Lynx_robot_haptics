function F = computeTextureForce(Fnq, character, velocity)
global PosEE;

Fk = norm(Fnq);
F = subs(character, ['Fn', 'v', 'pos'], [Fk, velocity, posEE]);