% function constants
% Load all the constants of the project . They are defined here and they 
% must not be changed in any other part of the code. They can be accessed 
% from any other function writing global and the name of the constants 
% to use
%
function constants

global E_T  B  R_L  R_R  LAMBDA_M  Q  R_observed  R_observer  SIMPRE
global COLIDX DEBLV

%% Simulation values

% Wheels and encoders data
E_T = 2048;
B   = 0.35;
R_L = 0.1;
R_R = 0.1;

% Threshold for the outlier detection
LAMBDA_M = chi2inv(0.999, 2);

% Covariance matrices for the noise processes
Q = diag([1^2 1^2 1^2]);                % Noise in the motion model
R_observed = diag([0.1^2 0.1^2]);       % Noise in the observations
R_observer = diag([0.1^2 0.1^2 0.1^2]); % Noise in the observations

%% File information

SIMPRE = 'simcl';   % Prefix for the simulation files

%% Plot information

COLIDX = ['g' 'r' 'b'];

%% Debug information

DEBLV = 0;