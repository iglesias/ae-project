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

global E_T  B  R_L  R_R  LAMBDA_M  Q  R

constants;

% Robot structures (initial covariances and poses)
% TODO parametrize this functions because probably we don't want all the robots
% starting on the same location
robot1 = init_robot();
robot2 = init_robot();

% Miscellaneous
% sensorpose      = zeros(3, 1);
total_outliers  = 0;
enc1            = zeros(2, 1);
enc2            = zeros(2, 1);
t               = 0;

%% Figures Initialization

dataset_basedir = 'Datasets/';
margin          = 5;

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

  n = values(10);
  if (n > 0)
    bearings = values(12:3:end);
    ranges   = values(13:3:end);
    ids      = values(11:3:end);
  else
    bearings = [];
    ranges   = [];
    ids      = [];
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

  % Compute the control signals of the robots

  robot1.u = calculate_odometry(denc1(1), denc1(2), E_T, B, R_R, R_L, delta_t1, ...
                                robot1.mu);
  robot2.u = calculate_odometry(denc2(1), denc2(2), E_T, B, R_R, R_L, delta_t2, ...
                                robot2.mu);

  z = [ranges'; bearings'];
  known_associations = ids';

  [robot1, outliers]  = ekf_localize( robot1, R, Q, z, ...
                                      known_associations, M, ... 
                                      LAMBDA_M, map_ids, i  );
  total_outliers      = total_outliers + outliers;

  % Plot the estimates
  if n > 0

    plot(robot1.mu(1), robot1.mu(2), 'rx')

    pcov = make_covariance_ellipses(robot1.mu, robot1.sigma);
    set(hcovs, 'xdata', pcov(1, :), 'ydata', pcov(2,:));
    title( sprintf('t = %d, total outliers = %d, current outliers = %d', ...
                    i, total_outliers, outliers) );
                
    axis( [xmin xmax ymin ymax] ) 

  end

  % Plot the true pose
  if n > 0        

    plot(truepose1(1), truepose1(2), 'gx');
    
    axis([xmin xmax ymin ymax]) 

  end

  % Plot the odometry
  if n > 0

    plot(odom1(1), odom1(2), 'bx');
    
    axis([xmin xmax ymin ymax]) 

  end

  drawnow

end % while

end
