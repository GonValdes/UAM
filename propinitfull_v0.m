% Initialisation File

% This file includes all the constants for the initialisation of the
% propeller 
% Written  by G Valdes - 30 Mar 2020

% based on A K Cooke - 6 Mar 07, updated 22 Nov 10

%% Forward propellers
% Load propeller geometry and data obtained in Qprop
load('forward_prop.mat')

% Define other parameters 
ForwProp.xyzright = -ForwProp.xyzrightCG + [simsetup.CG;0;0]; % location from body axes centre in metres. z positive downwards
ForwProp.xyzleft = -ForwProp.xyzleftCG + [simsetup.CG;0;0]; % location from body axes centre in metres

%Fixed or calculated from inputs
ForwProp.D   = 2*ForwProp.radius_tip;                  % propeller diameter in metres
ForwProp.beta075 = interpn(ForwProp.y_distribution/ForwProp.radius_tip,ForwProp.beta_distribution,0.75) ;%deg
ForwProp.type = 2; % 1 single, 2 coaxial
% ForwProp.solidity = 4*ForwProp.n_blades*mean(ForwProp.c_distr)/(3*pi*ForwProp.D);% solidity
ForwProp.Ap  = pi * (ForwProp.D^2)/4;           % propeller disc area
ForwProp.Sn   = 0;                   % maximum nacelle cross-sectional area
ForwProp.inertia = 8/15*2700*(0.3^5-0.29^5)+ForwProp.n_blades/3*1600*(ForwProp.D/2)^3*mean(ForwProp.c_distribution)*0.01; 

%% Back propeller
load('back_prop.mat')
% Define other parameters 
BackProp.xyz = -BackProp.xyzCG +[simsetup.CG ;0;0]; % location from body axes centre in metres

%Fixed or calculated from inputs
BackProp.D   = 2*BackProp.radius_tip;                  % propeller diameter in metres
BackProp.beta075 = interpn(BackProp.y_distribution/BackProp.radius_tip,BackProp.beta_distribution,0.75) ;%deg
BackProp.type = 2; % 1 single, 2 coaxial
% BackProp.solidity = 4*BackProp.n_blades*mean(BackProp.c_distr)/(3*pi*BackProp.D);% solidity
BackProp.Ap  = pi * (BackProp.D^2)/4;           % propeller disc area
BackProp.Sn   = 0;                   % maximum nacelle cross-sectional area
BackProp.inertia = 8/15*2700*(0.3^5-0.29^5)+BackProp.n_blades/3*1600*(BackProp.D/2)^3*mean(BackProp.c_distribution)*0.01; 

