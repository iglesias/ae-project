% function run_cl(simoutfile1, simoutfile2, mapfile)
%
% This function is the entrance point to the code, it runs the Collective
% Localization (CL) algorithm
%
% Inputs:
%           simoutfile1:  simulation file used for the EKF robot
%           simoutfile2:  simulation file used for the CL robot
%           mapfile:      file with map information
%
function run_cl(simoutfile1, simoutfile2, mapfile)

%% Parameter Initilization

global E_T  B  R_L  R_R  LAMBDA_M  Q  R_observed  R_observer

constants;

% Robot structures (initial covariances and poses)
% TODO parametrize this functions because probably we don't want all the robots
% starting on the same location
robot1 = init_robot(1);
robot2 = init_robot(2);

% Miscellaneous
% sensorpose      = zeros(3, 1);
total_outliers1 = 0;
total_outliers2 = 0;
enc1            = zeros(2, 1);
enc2            = zeros(2, 1);
t               = 0;

%% Figures Initialization

dataset_basedir = 'Datasets/same_map/';
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
fid1    = fopen( [dataset_basedir simoutfile1], 'r' );
fid2    = fopen( [dataset_basedir simoutfile2], 'r' );

if fid1 <= 0
  fprintf('Failed to open simoutput file %s\n', simoutfile1);
  return
end

if fid2 <= 0
  fprintf('Failed to open simoutput file %s\n', simoutfile2);
  return
end

flines1 = {};
while 1
  line = fgetl(fid1);
  if ~ischar(line)
    break
  end
  flines1 = {flines1{:} line};
end

flines2 = {};
while 1
  line = fgetl(fid2);
  if ~ischar(line)
    break
  end
  flines2 = {flines2{:} line};
end

fclose(fid1);
fclose(fid2);

%% Main Loop
i = 0;
while i < min( length(flines1), length(flines2) )

  i       = i + 1;

  % Read robot1's data

  line    = flines1{i};
  values  = sscanf(line, '%f');
  
  pt1          = t;
  t1           = values(1);
  delta_t1     = t1 - pt1;
  odom1        = values(2:4);
  penc1        = enc1;
  enc1         = values(5:6);
  denc1        = enc1 - penc1;
  truepose1    = values(7:9);

  n1 = values(10);
  
  if (n1 > 0)
    bearings1 = values(12:3:12+3*(n1-1));
    ranges1   = values(13:3:13+3*(n1-1));
    ids1      = values(11:3:11+3*(n1-1));
    x_diff_12       = values(11+3*n1);
    y_diff_12       = values(12+3*n1);
    theta_diff_12   = values(13+3*n1);
  else
    bearings1 = [];
    ranges1   = [];
    ids1      = [];
    x_diff_12       = [];
    y_diff_12       = [];
    theta_diff_12   = [];
  end

  % Read robot2's data
 
  line    = flines2{i};
  values  = sscanf(line, '%f');
  pt2          = t;
  t2           = values(1);
  delta_t2     = t2 - pt2;
  odom2        = values(2:4);
  penc2        = enc2;
  enc2         = values(5:6);
  denc2        = enc2 - penc2;
  truepose2    = values(7:9);
  
  n2 = values(10);
  if (n2 > 0)
    bearings2 = values(12:3:12+3*(n2-1));
    ranges2   = values(13:3:13+3*(n2-1));
    ids2      = values(11:3:11+3*(n2-1));
    x_diff_21       = values(11+3*n2);
    y_diff_21       = values(12+3*n2);
    theta_diff_21   = values(13+3*n2);
  else
    bearings2 = [];
    ranges2   = [];
    ids2      = [];
    x_diff_21       = [];
    y_diff_21       = [];
    theta_diff_21   = [];
  end

  % Compute the control signals of the robots

  robot1.u = calculate_odometry(denc1(1), denc1(2), E_T, B, R_R, R_L, delta_t1, ...
                                robot1.mu);
  robot2.u = calculate_odometry(denc2(1), denc2(2), E_T, B, R_R, R_L, delta_t2, ...
                                robot2.mu);

  % Localization algorithm for the first robot, the EKF robot

  z1 = [ranges1'; bearings1'];
  known_associations1 = ids1';

  [robot1, outliers1] = ekf_localize( robot1, R_observed, Q, z1, ...
                                      known_associations1, M, ... 
                                      LAMBDA_M, map_ids, i  );
  total_outliers1     = total_outliers1 + outliers1;
  
  % Localization algorithm for the second robot, the CL robot
  % TODO
  % cl_localize(robot2)
    
  if mod(i,20)==0
    z2 = [x_diff_21'; y_diff_21'; theta_diff_21'];
    [robot2, robot1] = cl_localize(robot2, Q, robot1, R_observer, z2);  
  end
    
  % Plot the estimates
  if n1 > 0

    plot(robot1.mu(1), robot1.mu(2), 'rx')

    pcov = make_covariance_ellipses(robot1.mu, robot1.sigma);
    set( hcovs, 'xdata', pcov(1, :), 'ydata', pcov(2,:) );
    title( sprintf('t = %d, total outliers = %d, current outliers = %d', ...
                    i, total_outliers1, outliers1) );
                
    axis( [xmin xmax ymin ymax] ) 

  end

  % Plot the true pose
  if n1 > 0        

    plot(truepose1(1), truepose1(2), 'gx');
    plot(truepose2(1), truepose2(2), 'go');
    
    axis([xmin xmax ymin ymax]) 

  end

  % Plot the odometry
  if n1 > 0

    plot(odom1(1), odom1(2), 'bx');
    plot(odom2(1), odom2(2), 'bo');
    
    axis([xmin xmax ymin ymax]) 

  end

  drawnow

end % while

end
