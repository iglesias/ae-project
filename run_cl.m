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

% Input parameters from other functions

global E_T  B  R_L  R_R Q  R_landmark  R_observer  SIMPRE COLIDX DEBLV

% Output parameteres

global M  MAP_IDS

constants;

% Robot structures (initial covariances and poses)
robots(1) = init_robot(1, nrobots, 2);
for r = 2:nrobots
  robots(r) = init_robot(r, nrobots, 2);
end

% Miscellaneous
trueposes       = zeros(nrobots, 3);
encs            = zeros(nrobots, 2);
odoms           = zeros(nrobots, 3);
ts              = zeros(nrobots, 1);  % Current timestamps
ids             = cell(1, nrobots);   % Variable length
bearings        = cell(1, nrobots);
ranges          = cell(1, nrobots);

%% Figures Initialization

dataset_basedir = 'Datasets/';
margin          = 15;

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
MAP_IDS = d(:, 1)';

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

% More initializations
i        = 0;
sigmas   = zeros(nrobots, niters, 3);
errposes = zeros(nrobots, niters, 3);

niters = 250;

while i < niters

  i = i + 1;
  
  % Read robots' data
  
  pts   = ts;     % Previous timestamps
  pencs = encs;   % Previous encoders signals
  
  for r = 1:nrobots
    line   = flines{r}{i};
    values = sscanf(line, '%f');
    
    ts(r)           = values(1);
    odoms(r, :)     = values(2:4);
    encs(r, :)      = values(5:6);
    trueposes(r, :) = values(7:9);
    
    n = values(10);
    if n > 0
      ids{r}       = values(11:3:11+3*(n-1));
      bearings{r}  = values(12:3:12+3*(n-1));
      ranges{r}    = values(13:3:13+3*(n-1));
    else
      bearings{r}  = [];
      ranges{r}    = [];
      ids{r}       = [];
    end
    
  end
  
  delta_ts = ts - pts;
  dencs    = encs - pencs;

  % Compute the control signals of the robots
  for r = 1:nrobots
    robots(r).u = calculate_odometry(dencs(r, 1), dencs(r, 2), E_T, B, ...
                                     R_R, R_L, delta_ts(r), robots(r).mu);
  end

  % Localization algorithm for EKF robots

  for ri = 1:nrobots
    
    switch robots(r).type
      
      case 1
        z                 = [ ranges{ri}'; bearings{ri}' ];
        params.known_asso = ids{ri}';
        robots = cl_localize(robots, ri, Q, R_landmark, z, params);
        
      case 2
        
        if ri == 4   % the observer
          
          % TODO Orthogonalize this part from the number of measurements
          if mod(i, 2) == 0
            rj = 1;
          else
            rj = 3;
          end
        
          l = ri;     % index for the observer
          m = rj;     % index for the observed
        
          % Generate a measurement
          z = genrobotmeas(trueposes(l, :)', trueposes(m, :)', R_observer);

          if DEBLV
            fprintf('>>>> Measurement from robot %d to robot %d\n', l, m);
          end

          params.l = l;
          params.m = m;
          robots = cl_localize(robots, ri, Q, R_observer, z, params);
          
        else
          robots = cl_localize(robots, ri, Q);
        end
          
      otherwise
        disp('Error in the definition of robot type!');
        return;
        
    end
    
  end
  
% %   if mod(i, 50) == 0
% %     keyboard
% %   end
  
  for r = 1:nrobots
    errposes(r, i, :) = trueposes(r, :)' - robots(r).mu;
    errposes(r, i, 3) = wrapToPi( errposes(1, i, 3) );
    sigmas(r, i, :)   = diag( robots(r).sigma(:, :, r) );
  end
  
  % Plot the estimates
    for r = 1:nrobots
      plot(robots(r).mu(1), robots(r).mu(2), [COLIDX(r) '*']);
    end
    
%     for r = 1:nrobots
%       pcov = make_covariance_ellipses(robots(r).mu, robots(r).sigma);
%       set( hcovs, 'xdata', pcov(1, :), 'ydata', pcov(2,:) );
%     end
    
  axis( [xmin xmax ymin ymax] ) 
  
  % Plot the true pose
  
  for r = 1:nrobots
    plot(trueposes(r, 1), trueposes(r, 2), [COLIDX(r) 'x']);
  end

  axis([xmin xmax ymin ymax]) 

  % Plot the odometry

  for r = 1:nrobots
    plot(odoms(r, 1), odoms(r, 2), [COLIDX(r) 'o']);
  end
    
  axis([xmin xmax ymin ymax]) 

  drawnow

end % while

if 1 % TODO add verbose
  
  for r = 1:nrobots
    figure;
    clf;
    
    subplot(3, 1, 1);
    hold on;
    plot( errposes(r, :, 1) );
    plot( 3*sqrt( sigmas(r, :, 1) ) );
    plot( -3*sqrt( sigmas(r, :, 1) ) );
    title( sprintf('error on x for the robot %d', r) );
    
    subplot(3, 1, 2);
    hold on;
    plot( errposes(r, :, 2) );
    plot( 3*sqrt( sigmas(r, :, 2) ) );
    plot( -3*sqrt( sigmas(r, :, 2) ) );
    title( sprintf('error on y for the robot %d', r) );
    
    subplot(3, 1, 3);
    hold on;
    plot( errposes(r, :, 3) );
    plot( 3*sqrt( sigmas(r, :, 3) ) );
    plot( -3*sqrt( sigmas(r, :, 3) ) );
    title( sprintf('error on theta for the robot %d', r) );
  end
  
end

end
