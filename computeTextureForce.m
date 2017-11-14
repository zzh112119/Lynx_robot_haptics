function F = computeTextureForce(Fnq, character, velocity)
global posEE;

F = [0;0;0];
Fk = norm(Fnq);
if Fk > 0
    F = double(subs(character, {'Fn', 'v', 'pos'}, {Fk, velocity, posEE'}));
end

end