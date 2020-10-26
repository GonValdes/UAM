% Script to run AVL and obtain aerodynamic data for the UAM project
% AVL documentation: http://web.mit.edu/drela/Public/web/avl/
% Created by Gonzalo Valdes April 2020
clear all;
clc; 
% All variables are in metric units except where stated

%% Initialise needed variables
%Change to path where all the scripts are found
addpath('D:\Cranfield\IRP\Coding\Clean')
dir.avl = 'D:\Cranfield\IRP\Coding\Clean';

filename.geo = 'a';

cd(dir.avl)
%% Define flight conditions

TAS = [300/3.6];%m/s
Altitude = [1000];%m

flight_con = GetFlightCond(Altitude,TAS);
% row- altitude, column-TAS
% clearvars -except flight_con filename dir

%% Initial set of parameters
% Wing 1-root chord(m) 2-tip chord(m) 3-le sweep(deg) 4-root aoa(deg) 5-X_le
% Canard 6-root chord(m) 7-semispan(m)  8-root aoa(deg) 9-zle(m) 
% CG position-10(m) 

par0 = [1.6,1.5,5,2,7];
par0 = [par0,[0.8,2,2,0.8]];
par0 = [par0,[5]];
%upper bound
ub =[2.2,2,25,2,8.5];
ub =[ub,[1,3,2.5,1]];
ub =[ub,[5.4]];
% lower bound
lb =[1,1,0,0,6];
lb =[lb,[0.7,1.5,0,0]];
lb =[lb,[4.6]];

%% Run AVL
%Determine value of the parameters that can vary
pars = (ub+lb)/2;

initialise = AVL_run(pars,flight_con{1,1},filename)