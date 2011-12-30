% function run_cl(nrobots, mapfile)
%
% This function is the entrance point to the code, it runs the Collective
% Localization (CL) algorithm
%
% Inputs:
%           nrobots:      number of robots to take part in the simulation
%           mapfile:      file with map information
%
function run_cl(nrobots, mapfile)

%% Parameter Initilization

global E_T  B  R_L  R_R  LAMBDA_M  Q  R_observed  R_observer  SIMPRE COLIDX

constants;

% Robot structures (initial covariances and poses)
% TODO parametrize this functions because probably we don't want all the robots
% starting on the same location
for i = 1:nrobots
  robots(i) = init_robot(i);
end

% Miscellaneous
% sensorpose      = zeros(3, 1);
total_outliers1 = 0;
enc1            = zeros(2, 1);
t               = 0;
trueposes       = zeros(nrobots, 3);
odoms           = zeros(nrobots, 3);

%% Figures Initialization

dataset_basedir = 'Datasets/';
margin          = 20;

d = load( [dataset_basedir mapfile] );

if 1  % TODO add verbose??

  mapfig = figure(1);   % Here it will be shown the map and the movement
  clf(mapfig);

  xmin = min( d(:, 2) ) - margin;
  xmax = max( d(:, 2) ) + margin;
  ymin = min( d(:, 3) ) - margin;
  ymax = max( d(:, 3) ) + margin;
  
  figure(mapfig);
  draw_landmark_map( [dataset_basedir mapfile] );
  hold on;
  axis( [xmin xmax ymin ymax] );
  title('Map and Movement of the Robots');

end

% ??

if 1  % TODO add verbose??
  figure(mapfig); 
  hcovs = plot(0, 0, 'r', 'erasemode', 'xor');
end

%% Read Simulation Files

M       = d(:, 2:3)';
map_ids = d(:, 1)';

% Open the simulation files
fids    = zeros(1, nrobots);
for i = 1:nrobots
  
  fids(i) = fopen( [dataset_basedir sprintf('%s%d.txt', SIMPRE, i)], 'r' );
  
  if fids(i) <= 0
    fprintf('Failed to open simoutput file %s\n', simoutfile1);
  end
  
end

% Read the information
flines = cell(1, nrobots);
for i = 1:nrobots
  flines{i} = {};
  while 1
    line = fgetl( fids(i) );
    if ~ischar(line)
      break
    end
    flines{i} = {flines{i}{:} line};
  end
end

% Close the files
for i = 1:nrobots
  fclose( fids(i) );
end

%% Main Loop

% Iterate as many times as the length of the shortest file
niters = length( flines{1} );
for i = 2:nrobots
  tmp    = length( flines{i} );
  niters = min(niters, tmp);
end
i = 0;
while i < niters

  i = i + 1;

  % Read robot i's data

  for r = 1:nrobots
    line   = flines{r}{i};
    values = sscanf(line, '%f');
    
    odoms(r, :)     = values(2:4);
    trueposes(r, :) = values(7:9);
  end
  
%   pt1          = t;
%   t1           = values(1);
%   delta_t1     = t1 - pt1;
%   odom1        = values(2:4);
%   penc1        = enc1;
%   enc1         = values(5:6);
%   denc1        = enc1 - penc1;
%   truepose1    = values(7:9);
% 
%   n1 = values(10);
%   
%   if (n1 > 0)
%     bearings1 = values(12:3:12+3*(n1-1));
%     ranges1   = values(13:3:13+3*(n1-1));
%     ids1      = values(11:3:11+3*(n1-1));
%   else
%     bearings1 = [];
%     ranges1   = [];
%     ids1      = [];
%   end
% 
%   % Compute the control signals of the robots
% 
%   robot1.u = calculate_odometry(denc1(1), denc1(2), E_T, B, R_R, R_L, ...
%                                 delta_t1, robot1.mu);
%   robot2.u = calculate_odometry(denc2(1), denc2(2), E_T, B, R_R, R_L, ...
%                                 delta_t2, robot2.mu);
% 
%   % Localization algorithm for the first robot, the EKF robot
% 
%   z1 = [ranges1'; bearings1'];
%   known_associations1 = ids1';
% 
%   [robot1, outliers1] = ekf_localize( robot1, R_observed, Q, z1, ...
%                                       known_associations1, M, ... 
%                                       LAMBDA_M, map_ids, i  );
%   total_outliers1     = total_outliers1 + outliers1;
%   
%   % Localization algorithm for the second robot, the CL robot
%   
%   % Recompute the measurement from one robot to the other
%   truepose1(3) = wrapToPi( truepose1(3) );
%   truepose2(3) = wrapToPi( truepose2(3) );
%   
%   c = [
%         cos( truepose2(3) )  -sin( truepose2(3) );
%         sin( truepose2(3) )   cos( truepose2(3) )
%       ];
%   a = c'*(truepose1(1:2)-truepose2(1:2));
%   b = wrapToPi( truepose1(3) - truepose2(3) );
%   z2 = [a ; b];
%   
%   if mod(i, 1) == 0
%     [robot2, robot1] = cl_localize(robot2, Q, robot1, R_observer, z2);
%   else
%     [robot2, robot1] = cl_localize(robot2, Q, robot1, R_observer);
%   end
% 
%   % Plot the estimates
%   if n1 > 0
% 
%     plot(robot1.mu(1), robot1.mu(2), 'rx');
%     plot(robot2.mu(1), robot2.mu(2), 'ro');
%     
%     pcov = make_covariance_ellipses(robot1.mu, robot1.sigma);
%     set( hcovs, 'xdata', pcov(1, :), 'ydata', pcov(2,:) );
%     pcov = make_covariance_ellipses(robot2.mu, robot2.sigma);
%     set( hcovs, 'xdata', pcov(1, :), 'ydata', pcov(2,:) );
%     title( sprintf('t = %d, total outliers = %d, current outliers = %d', ...
%                     i, total_outliers1, outliers1) );
%     
%                       
%     axis( [xmin xmax ymin ymax] ) 
% 
%   end
  
  % Plot the true pose
%   if n1 > 0        

    for r = 1:nrobots
      plot(trueposes(r, 1), trueposes(r, 2), [COLIDX(r) 'x']);
    end

    axis([xmin xmax ymin ymax]) 

%   end

  % Plot the odometry
%   if n1 > 0

    for r = 1:nrobots
      plot(odoms(r, 1), odoms(r, 2), [COLIDX(r) 'o']);
    end
    
    axis([xmin xmax ymin ymax]) 

%   end

  drawnow

end % while

end
