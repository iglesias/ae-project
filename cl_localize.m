% function [ output_args ] = cl_localize( input_args )
%
%
% Inputs: TODO
%           robot(t)        robot structure
%           Q               model noise's covariance
%           Rrobo           covariance in the noise of the measurements 
%                           between robots
%           z               observation from one robot to another
%
function robots = cl_localize(robots, observer_idx, Q, Rrobo, trueposes)

% Useful constants
nrobots = length(robots);

% Prediction or propagation phase
if robots(observer_idx).never_updated
  robots(observer_idx) = predict_init(robots(observer_idx), Q);
else
  robots(observer_idx) = predict(robots(observer_idx), Q, nrobots);
end

% Update phase
for ri = 1:nrobots
    

  if observer_idx ~= ri
    
    l = observer_idx;   % index for the observer
    m = ri;             % index for the observed
    
    % A bit ugly to generate the measurements here in the middle but...
    z = genrobotmeas(trueposes(l, :)', trueposes(m, :)', Rrobo);
    
    if ~robots(l).never_updated && ~robots(m).never_updated
      [robots(l) robots(m)] = predict_cross_terms( robots(l), robots(m) );
    end
    
    for rj = 1:nrobots
      
      if robots(rj).never_updated
        
        % Just update if rj is either the observer or the observed
        if rj == l
          robots(rj) = update_init_observer(robots(m), robots(l), Rrobo, z);
        elseif rj == m
          robots(rj) = update_init_observed(robots(m), robots(l), Rrobo, z);
        end
        
      else
        robots(rj) = update(robots, rj, l, m, Rrobo, z);
      end
      
    end
    
  end
  
end
  
end


