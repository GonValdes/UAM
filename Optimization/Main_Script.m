clear all;
clc; 
% All variables are in metric units except where stated

%% Establish the directories to work
%Define the path to each folder/directory
dir.work = '';%Directory where AVL, qprop and xfoil are found
dir.scripts_aux = '\Scripts';% Directory where all the scripts are found/scripts folder

dir.qprop = dir.work; %qprop.directory
dir.AVL = dir.qprop; %AVL.exe directory


addpath(dir.scripts_aux)
addpath(dir.work)

%Name for the running files, can be whatever
filename.qprop_results='try';
filename.geo = 'a';

cd(dir.work)
%% Load necessary data to run optimization
%Aerofoil dataset for qprop
%Neural networks response surface for AVL
%Back propeller response surface for optimization
load aerofoil_database.mat
load AVL_response_surfaces_cleanv4.mat
load BackProp.mat

%% Define flight conditions
TAS = [300/3.6];%m/s
Altitude = [1000];% m
flight_con = GetFlightCond(Altitude,TAS);

%% Initial set of parameters for airframe
%Define bounds for the airframe geometry
% Wing 1-root chord(m) 2-tip chord(m) 3-le sweep(deg) 4-root aoa(deg) 5-X_le
% Canard 6-root chord(m) 7-semispan(m)  8-root aoa(deg) 9-zle(m) 
% CG position-10(m) 

%upper bound
ub =[2.5,2,25,2,8.5];
ub =[ub,[1,3,2.5,1]];
ub =[ub,[5.4]];
% lower bound
lb =[1,1,0,0,6];
lb =[lb,[0.7,1.5,0,0]];
lb =[lb,[4.6]];

%% Initial set of parameters Propeller
%Define bounds for the forward propeller geometry
%12-tip radius(m) [[2-first y spline(%tip) 3-second y spline(%tip)]]
%13-pitch root(deg) 14-pitch 1st y point(%root) 15-pitch 2nd y point(%root) 16-pitch root(%root)
%17-root chord(%tip radius) 18-chord 1st y point(%root) 19-chord 2nd y point(%root) 20-chord tip(%root)
%21-thickness root(t/c) 22-thickness y1(%root) 23-thickness y2(%root) 24-thickness tip(%root)
%25-camber root(%c) 26-camber y1(%root) 27-camber y2(%root) 28-camber tip(%root)
%29-Thrust(N) [[31-TAS(m/s)-Fixed][33-altitude(m]- Fixed)] 
%30-forwpropCG(m) 31-backpropCG(m)

%lower bound
lb = [lb, [0.6999]];
lb = [lb, [20 0.4 0.4 0.4]];
lb = [lb,[0.01 0.4 0.4 0.4]];
lb = [lb,[6 0.4 0.4 0.4]];
lb = [lb,[0 0.4 0.4 0.4]];
lb = [lb,[950]];
lb = [lb,[2.2,6.5]];
%upper bound
ub = [ub,[0.7]];
ub = [ub, [50 2 2 2]];
ub = [ub,[0.12 1.6 1.6 1.6]];
ub = [ub,[20 1.6 1.6 1.6]];
ub = [ub,[9 1.6 1.6 1.6]];
ub = [ub,[1020]];
ub = [ub,[4.5,8]];

%% Optimization 
%Run optimization now. Three different stages according the procedure
%described in the thesis:
% Airframe optimization --> Propeller optimization --> Full aircraft
% optimization
%For each step, first run a genetic algorithm and then a pattern search algorithm

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Start with the only airframe
%Set options for the genetic algorithm
options = optimoptions('ga');
%Plotting
options = optimoptions(options,'PlotInterval',1);
options = optimoptions(options,'PlotFcn',{@gaplotbestindiv});%@gaplotscores
%Others
options = optimoptions(options,'FunctionTolerance',1e-1,'MaxGenerations',25,'MaxStallGenerations',25,'MaxTime',3600*10);
options = optimoptions(options,'UseParallel',false)
options = optimoptions(options,'PopulationSize',500);
options = optimoptions(options,'Display','off');
% options = optimoptions(options,'MutationFcn','crossoverheuristic');
% ,'MaxTime',43200

%Constrains
A = zeros(1,10); 
B = [];
A(1,1) = -1;A(1,2) = 1;B(1)=0;

%Seed the matrix with an initial airframe
seeding_mat =1.9393    1.9249    6.1656    2.0796    3.1167    1.2416    ...
    1.1087    2.3387  1.7497   0.8197   5.1097;1.1382    1.0104    6.9782    ...
    1.4640    3.2268    0.2147    0.9416    2.0339    1.7099    0.7091   5.2139]; 
options = optimoptions(options,'InitialPopulationMatrix',seeding_mat)

%Go to the directory where qprop is found
cd(dir.qprop)
%Define the cost function
fitness_funct = @(pars)Cost_function_opti_airframe(TAS,Altitude,pars,dir,filename,neural);
%You can test value with the seeding matrix
% Cost_function_opti_airframe(TAS,Altitude,seeding_mat(1,:),dir,filename,neural,CD0_data)

% Run the optimization
[x_airframe,fval_airframe,exitflag,output,population_airframe,scores_airframe]...
    = ga(fitness_funct,length(ub_airframe),A,B,[],[],lb_airframe,ub_airframe,[],options);

%Get a value to initialize the next cost function
[~,sol_airframe]=Cost_function_opti_airframe(TAS,Altitude,x_airframe,dir,filename,neural);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Now run optimization only for the propellers and the cg location of the
% propellers
%
%Define upper and lower bounds
ub_Prop = ub(11:30); ub_Prop(18) = [];
lb_Prop = lb(11:30); lb_Prop(18) = [];

%Define options
options = optimoptions('gamultiobj');
%Plotting
options = optimoptions(options,'PlotInterval',1);
options = optimoptions(options,'PlotFcn',{@gaplotpareto});
%Distance measure function, alculate distance either in function space (phenotype) or design space (genotype)
options.DistanceMeasureFcn = {@distancecrowding,'genotype'};
%Control distance measurement elitism. % of points in the Pareto front
options = optimoptions(options,'ParetoFraction',0.15);

%Stop Criteria
options = optimoptions(options,'FunctionTolerance',1e-1,'MaxGenerations',25,'MaxStallGenerations',25,'MaxTime',3600*20)
options = optimoptions(options,'UseParallel',false);
options = optimoptions(options,'PopulationSize',400);
options = optimoptions(options,'Display','iter');
% ,'MaxTime',43200
%Constraints
B=[];
A=zeros(2,19);
A(1,18) =-2/3;A(1,19) =-1;B(1)=-5/3*x_airframe(1,10);
A(2,18) =1;B(2)=x_airframe(1,10);

%Seeding matrix if desired
% seeding_mat = x(:,12:31);
% seeding_mat(:,18) = [];%fval_airframe/2;
% seeding_mat = x_Prop;
% options = optimoptions(options,'InitialPopulationMatrix',seeding_mat);

%Airframe data
airframe=x_airframe(1,:);

%Define the cost function
cd(dir.qprop)
fitness_funct = @(pars)Cost_function_optiv4neural_onlyProp_noTrim(TAS,Altitude,pars,airframe,dir,filename,AerofoilDataset,BackProp,Treq_prop)
%Run the optimization algorithm
[x_Prop,fval_Prop,exitflag,output,population,scores] = gamultiobj(fitness_funct,length(ub_Prop),A,B,[],[],lb_Prop,ub_Prop,@myconback,options);


