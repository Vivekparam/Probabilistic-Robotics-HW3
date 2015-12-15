%% STUDENT TODO: Need to fill this function that measures distances 
%% between states. This function should be able to take in a vector of states
%% and return their distance to a single state.
%% (a,b) -> can both be a single state  ==> d is a scalar
%% (a,b) -> "a" can be a vector of states, "b" a single state  ==> d is a vector of distances 
% For angle differences, use the function "angdiff" which returns an angle
% from -180 to +180 degrees.
function d = distance(a, b)
% INPUT:
% a - set of input poses [x,y,theta] => (N x 3), N can be > 1
% b - single pose [x,y,theta] => (1 x 3)
% OUTPUT:
% d - distance between states (scalar or (Nx1) vector)

% d = zeros(size(a,1));
d = sqrt(( a(:, 1)-b(1) ).^2 + (a(:, 2)-b(2)).^2);
end