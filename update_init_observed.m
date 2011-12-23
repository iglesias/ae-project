% function observed = update_init_observed(observed, observer, Rrobo, z)
% Performs the first update step for a robot. This robot has played the 
% role of observed in the measurement.
%
% Inputs:
%           observed(t):    robot structure
%           observer(t):    robot structure
%           Rrobo      :    3x3, covariance of the measurement noise 
%                           (between robots)
%           z(t)       :    3x1, measurement
%
% Outputs:
%           observed(t):    robot structure
%
function observed = update_init_observed(observed, observer, Rrobo, z)

% Compute H_tilde

J = [
      0  -1;
      1   0
    ];

p_observed = [ observed.mu_bar(1) observed.mu_bar(2) ]';
p_observer = [ observer.mu_bar(1) observer.mu_bar(2) ]';

H_tilde = [ 
            eye(2)          J*(p_observed - p_observer);
            zeros(1, 2)     1
          ];

% Compute R_tilde

C = [
      cos( observer.mu_bar(3) ) -sin( observer.mu_bar(3) );
      sin( observer.mu_bar(3) )  cos( observer.mu_bar(3) )
    ];

Gamma = [
          C             zeros(2, 1);
          zeros(1, 2)   1
        ];

R_tilde = Gamma*Rrobo*Gamma';

% Compute S_tilde using H_tilde and R_tilde
S_tilde = H_tilde*observer.sigma_bar*H_tilde' + observed_sigma_bar + ...
          R_tilde;
S_tinv  = S_tilde\eye(3);

% Update the mean of the belief
observed.mu = (eye(3) - observed.sigma_bar*S_tinv)*observed.mu_bar + ...
              observed.sigma_bar*S_tinv*(observer.mu_bar + Gamma*z);

% Update the uncertainty in the belief
observed.sigma = observed.sigma_bar - ...
                 observed.sigma_bar * S_tinv * observed.sigma_bar;

% Introduce the cross correlation term
observed.cross = observed.sigma_bar * S_tinv * H_tilde * observer.sigma_bar;

observed.lastUpdate = true;

end
