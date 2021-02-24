
%% Initialisation Urban Air Mobility
% Run this file before running simulink model
% All parameters in metric units unless stated otherwise
%
%   Gonzalo Valdes April 2020

% Landing gear my Mudassir Lone, 2018. Landing gear only included for
% take-off and landing in flight simulator. Does not generate forces nor
% moments.

clear all
clc

%% Initialise and run model
% Go to the folder where the Simulink model is found
cd('');

filename.model = 'Urban_Air_Mobility_v3.slx';
filename.AVL = 'AVL_data.mat';

disp('Loading aerodynamic data sets ...')
% Load look up table data
load(filename.AVL);

% clearvars -except flight_con BackProp ForwProp battery motor geo initialise

disp('Loading geometry and mass data ...')


simsetup.h = 1000;
% simsetup.M = 0.26;
simsetup.mass = 2000;
simsetup.CG = initialise.CG(1);
simsetup.g = 9.8065;

% Interpolate to obtain Inertia matrix

initialise.Ixx = interp1(initialise.CG,initialise.Ixx_data,simsetup.CG);
initialise.Iyy = interp1(initialise.CG,initialise.Iyy_data,simsetup.CG);
initialise.Izz = interp1(initialise.CG,initialise.Izz_data,simsetup.CG);
initialise.Ixz = interp1(initialise.CG,initialise.Ixz_data,simsetup.CG);

mass.Inertia = [initialise.Ixx 0 initialise.Ixz
    0 initialise.Iyy  0
     initialise.Ixz  0  initialise.Izz];

disp('Loading engine data ...')
% Load engine data
propinitfull_v0;

%% Dynamic pressure table 

initialise.alt = [ -1000 0 1000 2000 3000 4000 5000 6000 7000 8000 9000 10000 15000 ...
    20000];
initialise.dyn_visc = [ 1.821 1.789 1.758 1.726 1.694 1.661 1.628 1.595 1.561 1.527 ...
    1.493 1.458 1.422 1.422];
initialise.dyn_visc = initialise.dyn_visc*1e-5;

%% landing gear initialisation
disp('Loading landing gear data ...')

%Gear positions about CG in metres (x,y,z):
initialise.landinggear.gear_positions=[-1, -3.8, 3.2;   %main gear left
                                       -1,  3.8, 3.2;   %main gear right
                                        12,    0,   3.2];  %nose gear

%LG spring constants:
initialise.landinggear.x_static=0.5; %m
initialise.landinggear.K=30000*9.81/initialise.landinggear.x_static; %N/m, mass = 30000kg
initialise.landinggear.zeta=1/sqrt(2);
initialise.landinggear.C=2*initialise.landinggear.zeta*sqrt(initialise.landinggear.K*30000); %Ns/m, mass = 30000kg

%NG tyre constants:
initialise.landinggear.nosegear.mu_static_long=0.2;
initialise.landinggear.nosegear.mu_static_lat=0.4;
initialise.landinggear.nosegear.breakout_vel_lat=2; %m/sec
initialise.landinggear.nosegear.mu_kinetic_lat=0.1;

%MG tyre constants:
initialise.landinggear.maingear.mu_static_long=0.2;
initialise.landinggear.maingear.mu_static_lat=0.7;
initialise.landinggear.maingear.breakout_vel_lat=0.5; %m/sec
initialise.landinggear.maingear.mu_kinetic_lat=0.5;

%landing gear aero 
[initialise.landinggear.aero] = A320_UC(initialise.landinggear.gear_positions(1,3), ...
    initialise.landinggear.gear_positions(3,3));

%% Initial conditions
simsetup.initialXYZ = [0 0 -1000];
simsetup.initialUVW = [50 0 1];
simsetup.initialEuler = [0 0 0]*180/pi ;

simsetup.initialLatLong = [52.0732 -0.620928];% Cranfield