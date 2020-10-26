function [cost_funct,sol,sol2] = Cost_function_opti_airframe_2speeds(TAS,Altitude,TAS2,pars,dir,filename,neural)
% Cost function considering only the airframe. Trim achieved using only
% body aoa and elevator deflection. A second airspeed TAS2 is added to
% included a stall speed constrain, where trim must be achieved for
% aoa<10deg (linear aerodynamics)
%
% Inputs:
%   TAS and Altitude: Flight condition evaluated
%   pars: geometry inputs to define airframe geometry
%   dir: structure with the name of the directorios. Must include .AVL
%   neural: structure with the response surface for the aeroderivatives

%% Aerodynamic Block
% Obtain airframe geometry
geo = get_geo(pars)

%% Flight condition
% Obtain flight condition information

flight_con = GetFlightCond(Altitude,TAS);

cond.rho = flight_con{1,1}.rho;%flight_con{2}.rho;%kg/m3 density %0.6597
cond.mu = flight_con{1,1}.Visc;%flight_con{2}.DynVis;%kg/m.s dynamic viscosity %1.5951*10^-5
cond.a= flight_con{1,1}.a;%flight_con{2}.a; %m/s speed of sound %316.4
cond.Re = cond.rho*TAS*geo.mac_c/cond.mu;
cond.Mach = TAS/cond.a;
if TAS ==0
    cond.Re = cond.rho*2*geo.mac_c/cond.mu;
    cond.Mach = 2/cond.a;
end

%% Calculate Aeroderivatives with response surface
%Only for the airframe
AVL_input = [pars(1),pars(2),pars(3),pars(4),pars(5),pars(6),pars(7),pars(8),...
    pars(9),pars(10)];

initialise.CL0 = neural.CL0_neural(AVL_input');
initialise.CLa = neural.CLa_neural(AVL_input');
initialise.Cm0 = neural.Cm0_neural(AVL_input');
initialise.Cma = neural.Cma_neural(AVL_input');

initialise.CLdinA = neural.CLdinA_neural(AVL_input');
initialise.CmdinA = neural.CmdinA_neural(AVL_input');
initialise.CDdinA = neural.CDdinA_neural(AVL_input');

%% Calculate empirical drag correction 
[initialise.CD0,CD] = getCD0(geo,cond.Re,cond.Mach);

%% TRIM the aircraft
try
    %Get dynamic pressure
    qbar = 0.5*cond.rho*TAS^2;
    % Obtain initial estimation for angle of attack and elevator deflection
    aoa_0 = ((2000*9.81/(qbar*geo.S_w))-initialise.CL0)/initialise.CLa;
    if aoa_0>10*pi/180
        aoa_0=10*pi/180;
    end
    eta_0 = (-initialise.Cm0 - initialise.Cma*aoa_0)/initialise.CmdinA;
    %Obtain spanwise location of mean aerodynamic chord
    y_mac = ymac(geo);
    
    %Define trimming algorithm, initial point to run algorithm and options
    fun = @(x)trim_funct_airframe(x,initialise,geo,qbar,CD,y_mac);
    x0 = [aoa_0,eta_0];
    
    opts.FunctionTolerance = 50 ;
    opts.MaxIterations = 5000;
    opts.MaxFunctionEvaluations = 5000;
    opts.Diagnostics = 'off';
    opts.Display='off';
    %     opts.OptimalityTolerance = 5;
    % Run the trim algorithm
    [sol,~] = fsolve(fun,x0,opts);
    
    % With the obtained solution obtain the estimated required thrust=drag
    Thrust_req = qbar*geo.S_w*(initialise.CD0 +...
    geo.K*(initialise.CL0 + initialise.CLa*sol(1) + initialise.CLdinA*sol(2))^2 ...
    + abs(initialise.CDdinA*sol(2)));
    %Obtain the forces for that trimming
    F = trim_funct_airframe(sol,initialise,geo,qbar,CD,y_mac);
    

    %% Repeat process for the stall speed TAS 2
    qbar2 = 0.5*cond.rho*TAS2^2;
    
    aoa_02 = ((2000*9.81/(qbar2*geo.S_w))-initialise.CL0)/initialise.CLa;
    if aoa_02>10*pi/180
        aoa_02=10*pi/180;
    end
    eta_02 = (-initialise.Cm0 - initialise.Cma*aoa_02)/initialise.CmdinA;
    
    
    fun2 = @(x)trim_funct_airframe(x,initialise,geo,qbar2,CD,y_mac);
    x02 = [aoa_02,eta_02];
    
    opts.FunctionTolerance = 50 ;
    opts.MaxIterations = 50000;
    opts.MaxFunctionEvaluations = 50000;
    opts.Diagnostics = 'off';
    opts.Display='off';
    %     opts.OptimalityTolerance = 5;
    [sol2] = fsolve(fun2,x02,opts);
    
    Thrust_req2 = qbar2*geo.S_w*(initialise.CD0 +...
    geo.K*(initialise.CL0 + initialise.CLa*sol2(1) + initialise.CLdinA*sol2(2))^2 ...
    + initialise.CDdinA*abs(sol2(2)));
    
    F2 = trim_funct_airframe(sol2,initialise,geo,qbar2,CD,y_mac);

    %% Establish condition
    %In this case, angles are limited so aoa of attack and elevator deflection are small
    %-sol = [aoa(rad), elevator deflection(deg)] at flight condition 1
    %-sol2 = [aoa(rad), elevator deflection(deg)] at flight condition 2/stall constrain
    %-F is used to define the tolerance on the accuracy of the trimming forces
    if (abs(sol(1))>4*pi/180) || (sol(1)<-2*pi/180) || (abs(sol(2))>1) || (abs(F(1))>50) || (abs(F(2))>50)...
            || (abs(sol2(1))>10.5*pi/180) || (abs(sol2(2))>35) || (abs(F2(1))>50) || (abs(F2(2))>50)
        
        %This is used to check what is failing. Is the elevator angle to
        %high? The aoa? Both?
         
        aa=0;bb=0;cc=0;dd=0;
        if abs(sol2(1))>10.5*pi/180
            aa= abs(sol2(1))-10.5*pi/180;
        elseif abs(sol2(2))>35
            bb= abs(sol2(2))-35;
        end
        if abs(sol(1))>4*pi/180
            cc= (abs(sol(1))-4*pi/180);
        elseif (abs(sol(2))>1)
            dd= abs(sol(2))-1;
        end
        if sol(1)<-2*pi/180
            ee= abs(sol(1))-2*pi/180;
        end
        %Use this information to define the value of the constrains
        if abs(sol(1))>4*pi/180 || (abs(sol(2))>1) || (abs(sol2(1))>10.5*pi/180) || (abs(sol2(2))>35) 
            cost_funct = [8000,8000]+aa*1800/pi+bb*10+cc*1800/pi+dd*10+ee*1800/pi;
        else
            cost_funct = [90000,90000];
        end
    else
        %Cost function
        cost_funct = [Thrust_req,Thrust_req2];
    end
catch
    cost_funct = [100000,100000];
    
    fclose('all');
    if isfile(strcat(filename.qprop_results,'.txt'))
        delete(strcat(filename.qprop_results,'.txt'));
    end
end
end


function y_mac = ymac(geo)
%Obtain spanwise location of mean aerodynamic chord
A= (geo.Ct_w-geo.Cr_w)/geo.b_w;
B= geo.Cr_w;

y_mac = A*geo.mac_w+B;
end