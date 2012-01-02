% function robots = cl_localize(robots, idx, Q, Rrobo, z, l, m)
%
% Inputs:
%           robots          array of robot structures
%           idx             1x1, index in robots for the robot to perform 
%                           CL localization
%           Q               model noise's covariance
%           Rrobo           3x3, covariance in the noise of the  
%                           measurements between robots
%           z               3x1, observation from one robot to another
%           l               1x1, index in robots for the observer
%           m               1x1, index in robots for the observed
% Outputs          
%           robots          array of robot structures
%
function robots = cl_localize(robots, idx, Q, Rrobo, z, params)

global DEBLV

% Useful constants
nrobots = length(robots);

% Prediction or propagation phase
if robots(idx).never_updated
  
  if DEBLV
    fprintf('>>>> Prediction_init robot %d\n', idx);
  end
  robots(idx) = predict_init(robots(idx), Q);

else
  
  if DEBLV
    fprintf('>>>> Prediction robot %d\n', idx);
  end
  robots(idx) = predict(robots(idx), Q, nrobots);
  
end

if DEBLV
  fprintf('>>>> Update robot %d\n', idx);
end

% Update phase
if nargin > 3

  switch robots(idx).type
    case 1
      robots(idx) = landmark_update(robots(idx), Rrobo, z, ...
                                    params.known_assoc);
    case 2
      robots = robot_update(robots, Rrobo, z, params.l, params.m);
  end
          
else

  if DEBLV
    fprintf('>>>>>>>> With no measurement\n');
  end
  robots(idx).mu               = robots(idx).mu_bar;
  robots(idx).sigma(:, :, idx) = robots(idx).sigma_bar(:, :, idx);
  

end

end


