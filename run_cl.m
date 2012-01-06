% function run_cl(simoutfile1, simoutfile2, mapfile)
%
% This function is the entrance point to the code, it runs the Collective
% Localization (CL) algorithm
%
% Inputs:
%           simoutfile1     simulation file used for the EKF robot
%           simoutfile2     simulation file used for the CL robot
%           mapfile         file with map information
%           mode            1 - continuous communication between robots
%                           2 - no communication
%
function run_cl(simoutfile1, simoutfile2, mapfile, mode)

%% Parameter Initilization

global E_T  B  R_L  R_R  LAMBDA_M  Q  R_observed  R_observer

constants;

% Robot structures (initial covariances and poses)
% TODO parametrize this functions because probably we don't want all the robots
% starting on the same location
robot1 = init_robot(1);
robot2 = init_robot(2);

% Miscellaneous
total_outliers1 = 0;
enc1            = zeros(2, 1);
enc2            = zeros(2, 1);
t               = 0;

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

% Initialization
i = 0;
niters     = min( length(flines1), length(flines2) );
errposes1  = zeros(niters, 3);
errposes2  = zeros(niters, 3);
sigmas1    = zeros(niters, 3);
sigmas2    = zeros(niters, 3);

while i < niters

  i = i + 1;
  
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
  else
    bearings1 = [];
    ranges1   = [];
    ids1      = [];
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
  else
    bearings2 = [];
    ranges2   = [];
    ids2      = [];
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
  
  % Recompute the measurement from one robot to the other
  truepose1(3) = wrapToPi( truepose1(3) );
  truepose2(3) = wrapToPi( truepose2(3) );
  
  c = [
        cos( truepose2(3) )  -sin( truepose2(3) );
        sin( truepose2(3) )   cos( truepose2(3) )
      ];
  a = c'*(truepose1(1:2)-truepose2(1:2));
  b = wrapToPi( truepose1(3) - truepose2(3) );
  z2 = [a ; b];
  
  if mode == 1
    [robot2, robot1] = cl_localize(robot2, Q, robot1, R_observer, z2);
  else
    [robot2, robot1] = cl_localize(robot2, Q, robot1, R_observer);
  end
    
  % Store data for final observations
  errposes1(i, :) = truepose1 - robot1.mu;
  errposes1(i, 3) = wrapToPi( errposes1(i, 3) );
  errposes2(i, :) = truepose2 - robot2.mu;
  errposes2(i, 3) = wrapToPi( errposes2(i, 3) );

  sigmas1(i, :)   = diag(robot1.sigma);
  sigmas2(i, :)   = diag(robot2.sigma);
  
  % Plot the estimates
  if n1 > 0

    plot(robot1.mu(1), robot1.mu(2), 'rx');
    plot(robot2.mu(1), robot2.mu(2), 'ro');
    
    pcov = make_covariance_ellipses(robot2.mu, robot2.sigma);
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

if 1 % TODO add verbose
  
  robot1_fig = figure;
  robot2_fig = figure;
  clf;

  figure(robot1_fig);
  subplot(3, 1, 1);
  hold on;
  plot( errposes1(:, 1) );
  plot( 3*sqrt( sigmas1(:, 1) ) );
  plot( -3*sqrt( sigmas1(:, 1) ) );
  title('error on x for the robot 1');
  
  figure(robot2_fig);
  subplot(3, 1, 1);
  hold on;
  plot( errposes2(:, 1) );
  plot( 3*sqrt( sigmas2(:, 1) ) );
  plot( -3*sqrt( sigmas2(:, 1) ) );
  title('error on x for the robot 2');
  
  figure(robot1_fig);
  subplot(3, 1, 2);
  hold on;
  plot( errposes1(:, 2) );
  plot( 3*sqrt( sigmas1(:, 2) ) );
  plot( -3*sqrt( sigmas1(:, 2) ) );
  title('error on y for the robot 1');

  figure(robot2_fig);
  subplot(3, 1, 2);
  hold on;
  plot( errposes2(:, 2) );
  plot( 3*sqrt( sigmas2(:, 2) ) );
  plot( -3*sqrt( sigmas2(:, 2) ) );
  title('error on y for the robot 2');
  
  figure(robot1_fig);
  subplot(3, 1, 3);
  hold on;
  plot( errposes1(:, 3) );
  plot( 3*sqrt( sigmas1(:, 3) ) );
  plot( -3*sqrt( sigmas1(:, 3) ) );
  title('error on theta for the robot 1');

  figure(robot2_fig);
  subplot(3, 1, 3);
  hold on;
  plot( errposes2(:, 3) );
  plot( 3*sqrt( sigmas2(:, 3) ) );
  plot( -3*sqrt( sigmas2(:, 3) ) );
  title('error on theta for the robot 2');
  
end

end
