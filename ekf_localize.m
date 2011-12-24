% function [robot, outliers] = ekf_localize(robot, R, Q, z, known_assoc, M,
%                                           Lambda_M, map_ids_true, t)
% This function should perform one iteration of EKF localization using batch
% update.
%
% Inputs:
%           robot:                robot structure
%           R:                    3X3
%           Q:                    2X2
%           z:                    2Xn
%           known_assoc:          1Xn
%           M:                    2XN
%           Lambda_M:             1X1
%           map_ids_true:         1xN
%           t:                    1X1
% Outputs:
%           robot:               robot structure
%           outliers:            1X1
%
function [robot, outliers] = ekf_localize(robot, R, Q, z, known_assoc, M, Lambda_M, ...
                                          map_ids_true, t)

% TODO decide which predict is the one to be used
robot = predict(robot, R);

n         = size(z, 2);
outliers  = 0;

i  = 0;

% TODO adapt this to the robot structure
[c, outlier, nu_bar, H_bar] = batch_associate(mu_bar, sigma_bar, z, M, ...
                                              Lambda_M, Q);

if sum(outlier)
  fprintf('warning, %d measurements were labelled as outliers, t = %d', ...
          sum(outlier), t);
end

valid_idxs = find(~outlier);  % indices of inliers
ix = [2*(valid_ixs-1)+1;2*(valid_ixs-1)+2];
ix = ix(:); 
nu_bar = nu_bar(ix);
H_bar = H_bar(ix,:);
n = length(valid_ixs);
Q_bar = zeros(2*n,2*n);
for i=1:n
    ii= 2*i + (-1:0);
    Q_bar(ii,ii) = Q;
end

% TODO adapth this to the robot structure
[mu,sigma] = batch_update(mu_bar,sigma_bar,H_bar,Q_bar,nu_bar);   
outliers = sum(outlier);

end
