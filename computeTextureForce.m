function F = computeTextureForce(Fn, character, velocity)
global PosEE;

F = subs(character, ['Fn', 'velocity', 'posEE'], [Fn, velocity, posEE]);