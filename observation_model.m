% function h = observation_model(x, M, j)
% This function is the implementation of the h function.
% The bearing should lie in the interval [-pi,pi)
%
% Inputs:
%           x(t)        3X1
%           M           2XN
%           j           1X1
%
% Outputs:  
%           h           2X1
%
function h = observation_model(x, M, j)

h = [
        sqrt( ( M(1,j)-x(1) )^2 +( M(2,j)-x(2) )^2 );
        atan2( M(2,j)-x(2), M(1,j)-x(1) ) - x(3);
    ];

% Normalize the bearing so it lies it [-pi,pi)
if h(2) >= pi
    h(2) = h(2)-2*pi;
elseif h(2) < -pi
    h(2) = h(2)+2*pi;
end

end