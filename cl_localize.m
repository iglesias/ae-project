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
function robots = cl_localize(robots, idx, Q, Rrobo, z, l, m)

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

  if ~robots(l).never_updated && ~robots(m).never_updated
    [robots(l) robots(m)] = predict_cross_terms( robots(l), robots(m) );
  end

  for rj = 1:nrobots

    if robots(rj).never_updated

      % Just update if rj is either the observer or the observed
      if rj == l
        
        if DEBLV
          fprintf('>>>>>>>> Update_init_observer robot %d\n', rj);
        end
        robots(rj) = update_init_observer(robots(m), robots(l), Rrobo, z);
        
      elseif rj == m
        
        if DEBLV
          fprintf('>>>>>>>> Update_init_observed robot %d\n', rj);
        end
        robots(rj) = update_init_observed(robots(m), robots(l), Rrobo, z);
        
      end

    else
      
      if DEBLV
        fprintf('>>>>>>>> Update robot %d\n', rj);
      end
      robots(rj) = update(robots, rj, l, m, Rrobo, z);
      
    end

  end
    
else
  
  if DEBLV
    fprintf('>>>>>>>> With no measurement\n');
  end
  robots(idx).mu    = robots(idx).mu_bar;
  robots(idx).sigma = robots(idx).sigma_bar;
  
end

end


