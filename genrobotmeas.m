% function z = genrobotmeas(observer_pose, observed_pose, Rrobo)
%
% Generate a measurement from the robot observer to the robot observed
%
% Inputs:
%           observer_pose   3x1
%           observed_pose   3x1
%           Rrobo           3x3, cov of the measurement noise
%
% Outputs:
%
%           z               3x1, the measurement
%
function z = genrobotmeas(observer_pose, observed_pose, Rrobo)

% Sanity check, probability not necessary
observer_pose(3) = wrapToPi( observer_pose(3) );
observed_pose(3) = wrapToPi( observed_pose(3) );

C = [
      cos( observer_pose(3) )   -sin( observer_pose(3) );
      sin( observer_pose(3) )    cos( observer_pose(3) )
    ];

A = C'*( observed_pose(1:2) - observer_pose(1:2) );
B = wrapToPi( observed_pose(3) - observer_pose(3) );

z = [A ; B] + sqrt(Rrobo)*rand(3,1);

end

