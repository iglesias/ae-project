% function robot = update(robot, observer, observed)
%
% Inputs
%           robots      array of robot structures
%           i           index in robots for the robot to update
%           l           index in robots for the observer
%           m           index in robots for the observed
%           Rrobo       3x3, covariance of the measurement noise
%           z           3x1, measurement
%
% Outputs
%           robot       robot structure
%           
function robot = update(robots, i, l, m, Rrobo, z)

% Useful constants
nrobots = length(robots);

% Compute H

J = [
      0 -1;
      1  0
    ];

p_observed = [ robots(m).mu_bar(1) robots(m).mu_bar(2) ]';
p_observer = [ robots(l).mu_bar(1) robots(l).mu_bar(2) ]';

H = [ 
        eye(2)          J*(p_observed - p_observer);
        zeros(1, 2)     1
    ];

% Compute R

C = [
      cos( robots(l).mu_bar(3) ) -sin( robots(l).mu_bar(3) );
      sin( robots(l).mu_bar(3) )  cos( robots(l).mu_bar(3) )
    ];

Gamma = [
          C             zeros(2, 1);
          zeros(1, 2)   1
        ];

R = Gamma*Rrobo*Gamma';

% Compute S using H and R
S = H*robots(l).sigma_bar(:, :, l)*H' - ...
    robots(m).sigma_bar(:, :, l)*H' - H*robots(l).sigma_bar(:, :, m) ...    
    + robots(m).sigma_bar(:, :, m) + R;
  
Sinv = S \ eye(3);

% Compute the Kalman gain
K = ( robots(i).sigma_bar(:, :, m) - robots(i).sigma_bar(:, :, l)*H' )*Sinv;

% Update the mean of the belief
robots(i).mu =  robots(i).mu_bar + ...
                  K*( Gamma*z - (robots(m).mu_bar - robots(l).mu_bar) );

% Update the uncertainty in the belief and the cross terms
for r = 1:nrobots
  
    robots(i).sigma(:, :, r) = robots(i).sigma_bar(:, :, r) - ...
                               ( robots(i).sigma_bar(:, :, m) - ...
                                 robots(i).sigma_bar(:, :, l)*H' )* ...
                               Sinv* ...
                               ( robots(m).sigma_bar(:, :, r) - ...
                                 H*robots(l).sigma_bar(:, :, r) );
                             
end

robots(i).last_update = true;

% Return value              
robot = robots(i);
              
end

