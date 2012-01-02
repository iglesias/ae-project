% function robot = landmark_update(robot, R, z, known_asso, M, lambda_m, ...
%                                   MAP_IDS)
% This function performs one iteration of EKF localization using batch
% update.
%
% Inputs:
%           robot:                robot structure
%           R:                    3X3
%           Q:                    2X2
%           z:                    2Xn
%           known_asso:           1Xn
%           Lambda_M:             1X1
% Outputs:
%           robot:               robot structure
%           outliers:            1X1
%
function robot = landmark_update(robot, R, z, known_asso)

global MAP_IDS  M                                
                                
[c, outlier, nu_bar, H_bar] = batch_associate(robot, z, M, R);

if sum(outlier)
  fprintf('%d measurements were labelled as outliers', ...
          sum(outlier));
end

map_ids = zeros( 1, size(z, 2) );
for i = 1:size(z, 2)
  map_ids(i) = find( MAP_IDS == known_asso(i) );
  if map_ids(i) ~= c(i)
    fprintf('%dth measurement of landmark %d incorrectly associated ', ...
            'to landmark %d, t = %d', i, map_ids(i), c(i));
  end
end

valid_ixs   = find(~outlier);  % indices of inliers
ix          = [2*(valid_ixs - 1) + 1; 2*(valid_ixs - 1) + 2];
ix          = ix(:);

n       = length(valid_ixs);
nu_bar  = nu_bar(ix);
H_bar   = H_bar(ix, :);
R_bar   = zeros(2*n, 2*n);
for i = 1:n
    ii = 2*i + (-1:0);
    R_bar(ii, ii) = R;
end

robot = batch_update(robot, H_bar, R_bar, nu_bar);

robot.outliers = robot.outliers + sum(outlier);

if sum(sum(robot.sigma~=robot.sigma'))
    display('warning, sigma is not symmetric');
end

end
