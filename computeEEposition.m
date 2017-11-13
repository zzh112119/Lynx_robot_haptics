% Compute end effector position based on the current configuration

% Fill in the necessary inputs
function posEE = computeEEposition()

global qs;

% Fill this in
posEE = [0;0;0];
[X, T] = updateQ(qs);
posEE = X(4, :);
end