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
function robots = cl_localize(robots, Q, Rrobo, z, params)

global DEBLV

% Useful constants
nrobots = length(robots);

% Prediction or propagation phase
for r = 1:nrobots

  if robots(r).never_updated

    if DEBLV
      fprintf('>>>> Prediction_init robot %d\n', r);
    end
    robots(r) = predict_init(robots(r), Q);

  else

    if DEBLV
      fprintf('>>>> Prediction robot %d\n', r);
    end
    robots(r) = predict(robots(r), Q, nrobots);

  end

end
  
if DEBLV
  fprintf('>>>> Update robot %d\n', r);
end

% Update phase
if nargin > 4

  % TODO Discern type of measurement and call the corresponding update
% %   switch robots(idx).type
% %     case 1
% %       robots(idx) = landmark_update(robots(idx), Rrobo, z, ...
% %                                     params.known_assoc);
% %     case 2
% %       robots = robot_update(robots, Rrobo, z, params.l, params.m);
% %   end

  robots = robot_update(robots, Rrobo, z, params.l, params.m);

else

  if DEBLV
    fprintf('>>>>>>>> With no measurement\n');
  end
  
  for r = 1:nrobots
    robots(r).mu    = robots(r).mu_bar;
    robots(r).sigma = robots(r).sigma_bar;
  end
  
end

end


