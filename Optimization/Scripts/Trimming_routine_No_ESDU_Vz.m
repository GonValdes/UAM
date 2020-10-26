function [sol_final,Preq,P_req_forw,P_req_back] = Trimming_routine_No_ESDU_Vz(TAS,Vz,Altitude,pars,pars_BackProp,dir,filename,AerofoilDataset,neural)
% Function to Trim the aircraft at a certain flight condition. Based on a
% brute force algorithm.

%Propeller angle of attack effect on forces and moments calculated using ESDU 89047, assuming
%counter-rotating propellers

% Inputs:
%   TAS and Altitude: Flight condition evaluated
%   Vz:Vertical airspeed m/s
%   pars: geometry inputs to define airframe geometry
%   dir: structure with the name of the directorios. Must include .AVL
%   AerofoilDataset: Aerofoil aerodynamic characteristics dataset obtained with XFOIL 
%   BackProp: Response surface for the back propellers
%   neural:Data for the neural network response surface
%   
% Outputs:
%   P: Power for forward(forw) and back propellers and the total
%   sol_final: trimming angles


%% Aerodynamic Block
% Obtain airframe geometry
[geo,initialise] = get_geo(pars);

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
AVL_input = [pars(1),pars(2),pars(3),pars(4),pars(5),pars(6),pars(7),pars(8),...
    pars(9),pars(10)];

initialise.CL0 = neural.CL0_neural(AVL_input');
initialise.CLa = neural.CLa_neural(AVL_input');
initialise.Cm0 = neural.Cm0_neural(AVL_input');
initialise.Cma = neural.Cma_neural(AVL_input');

initialise.CLdinA = neural.CLdinA_neural(AVL_input');
initialise.CmdinA = neural.CmdinA_neural(AVL_input');
initialise.CDdinA = neural.CDdinA_neural(AVL_input');


%% Calculate empirical drag correction (Cause Xfoil drag correction proved to be useless...)
[initialise.CD0,CD] = getCD0(geo,cond.Re,cond.Mach);

%% Propeller Block
%Obtain forward propeller data
[prop,prop_cg_loc,prop_cg_locback] = get_prop(pars,initialise);

%% Calculate BackPropt Thrust
%Obtain back propeller data
BackProp = get_BackProp(pars_BackProp);

%% Trimming initial estimation
% Calculate flight path angle gamma in rad
if TAS == 0
    gamma = pi/2;
else
    gamma = asin(Vz/TAS);
end
% Define approximated initial conditions, an initial estimation for angle
% of attack, elevator deflection and thrust value
qbar = 0.5*cond.rho*TAS^2;
pitch_0 = ((2000*9.81/(qbar*geo.S_w))-initialise.CL0)/initialise.CLa+gamma;
if pitch_0>11*pi/180
    pitch_0=11*pi/180;
end
eta_0 = (-initialise.Cm0 - initialise.Cma*pitch_0)/initialise.CmdinA;
if eta_0>15
    eta_0=15;
end
T_0 = qbar*geo.S_w*(initialise.CD0 + geo.K*(initialise.CL0+initialise.CLa*pitch_0)^2)/2;
if isnan(T_0)
    T_0=2000*9.81*0.35/2;
end

%% TRIM

%Initialize some of the values
q_prop = 0;%cond.rho*rps^2*(prop.radius_tip*2)^4*CZ_alpha; % Forces(3) = cond.rho*rps^2*(prop.radius_tip*2)^4*CZ_alpha*alpha;
y_mac = ymac(geo);

% Define options for trimming algorithm
fun = @(x)trim_funct_estimation_BackProp_Vz(x,gamma,initialise,geo,qbar,q_prop,prop_cg_loc,prop_cg_locback,CD,y_mac);
opts = optimoptions('fsolve');
opts = optimoptions(opts,'Algorithm','Levenberg-Marquardt');%'trust-region-dogleg');
opts = optimoptions(opts,'FunctionTolerance' ,0.0001);
opts = optimoptions(opts,'MaxIterations',10000);
opts = optimoptions(opts,'MaxFunctionEvaluations', 10000);
opts = optimoptions(opts,'Diagnostics','off');
opts = optimoptions(opts,'Display','off');
%         opts = optimoptions(opts,'OptimalityTolerance', 0.05);

%Define the 5 control variables 
% 1 Thrust ForwProp (N) 2 Pitch angle body(rad) 3 Tilt(rad) 4 Elevator(rad) 5 Thrust BackProp(N) 
% Optimization ranges for thos variables. Multipliers added for better
% convergence of the solution, giving more importance to some variables
T_range=[T_0:1000:5000]/10000;
pitch_range = [0 5 10]*pi/180/1000;
tilt_range=[0:10:90]*pi/180/10;
eta_range=[0 15]*pi/180;
T_backprop_range = [0:2000:14000]/10000;

%Iteration to obtain trimming values
sols=[];
P_req_forwx=[];
P_req_backx=[];
P_reqx=[];

counter=1;
tic
for i =1:length(T_range)
    for j=1:length(pitch_range)
        for k =1:length(tilt_range)
            for l =1:length(eta_range)
                for m=1:length(T_backprop_range)
                    %First value for the algorithm to iterate and converge
                    x0 = [T_range(i),pitch_range(j),tilt_range(k),eta_range(l),T_backprop_range(m)];
                    %Running the trimming algorithm
                    [sol] = fsolve(fun,x0,opts);
                    %Obtain the forces value and the solution
                    F = trim_funct_estimation_BackProp_Vz(sol,gamma,initialise,geo,qbar,q_prop,prop_cg_loc,prop_cg_locback,CD,y_mac);
                    %reapply multipliers
                    sol(4)=sol(4)*180/pi;
                    sol(3)=sol(3)*10;
                    sol(2)=sol(2)*1000;
                    
                    %Condition for the validity of the solution
                    if (abs(F(1))>100) || (abs(F(2))>100) || (abs(F(3))>100)|| sol(1) < 0 || (sol(2)-gamma) > 10*pi/180 || abs(sol(4)) > 30|| sol(5) < 0 
                        continue % skip if conditions are not met
                    else
                        sol(1)=sol(1)*10000;
                        sol(5)=sol(5)*10000;
                        
                        %Calculate Power and save it to a vector/ matrix
                        %for each valid solution
                        try
                            run.V = TAS;
                            run.T =  sol(1);
                            [~,~,~,~,~]=qprop_analysisv3...
                                (prop,cond,dir,run,filename,AerofoilDataset);
                            [Q,rps] = get_TQtables(run,filename,cond,prop);
                            
                            delete(strcat(filename.qprop_results,'.txt'));
                            
                            runback.V = TAS;
                            runback.T = sol(5);
                            [~,~,~,~,~]=qprop_analysisv3...
                                (BackProp,cond,dir,runback,filename,AerofoilDataset);
                            [Qback,rpsback] = get_TQtables(runback,filename,cond,prop);
                            fclose('all');
                            delete(strcat(filename.qprop_results,'.txt'));
                            
                            P_req_forwx(counter,:) = Q*rps*4*pi;
                            P_req_backx(counter,:) = Qback*rpsback*2*pi;
                            P_reqx(counter,:) = P_req_forwx(counter,:) + P_req_backx(counter,:) ;%2 forward propellers + back prop
                            
                        catch
                            P_reqx(counter,:) = 10^10;
                            P_req_backx(counter,:) = 10^10;
                            P_req_forwx(counter,:) = 10^10;
                        end
                        
                        sols(counter,:)= sol;
                        
                        counter=counter+1;
                        
                    end
                end
            end
        end
    end
end

% Choose the minimum Prequired value
if isempty(sols)
    sol_final=[NaN,NaN,NaN,NaN,NaN];
    Preq = NaN;
    P_req_back = NaN;
    P_req_forw = NaN;
else
    
    [Preq,index] = min(P_reqx);
    sol_final = sols(index(1),:);
    P_req_back = P_req_backx(index(1));
    P_req_forw = P_req_forwx(index(1));
end

% sols_placa =sol_final;sols_placa(1)=sols_placa(1)/100
% trim_funct_estimation(sols_placa,initialise,geo,qbar,q_prop,prop_cg_loc,CD,y_mac)
end

function CZ_alpha = prop_correction(CT,pitch,J,sigma_e)
% pitch:    Prop pitch angle referred to zero-lift line of section 0.75R (deg)
if J ~= 0
    % ESDU 89047
    Tc = CT * 8/(pi * J^2);
    kb = tan(pi/180*(pitch - 5))/(sigma_e * cos(pi/180*(pitch - 5)));
    
    CZ_alpha = -((3.86 * sigma_e)/(1 + sigma_e)) * sin(pi/180*(pitch + 9))*...
        (1 + ((3 * Tc/8)/sqrt(1 + (2*Tc/3))));
    CZ_alpha = CZ_alpha * (CT/Tc);
    Cn_alpha = 0;
    
else
    Tc=0;
    
    CZ_alpha =0;
    Cn_alpha =0;
end
end

function y_mac = ymac(geo)
A= (geo.Ct_w-geo.Cr_w)/geo.b_w;
B= geo.Cr_w;

y_mac = A*geo.mac_w+B;
end