function [cost_funct,sol,Q,rps,Q2,rps2,initialise,geo] = Cost_function_optiv5neural(TAS,TAS2,Altitude,pars,dir,filename,AerofoilDataset,neural,BackProp,sol_airframe)
%Cost function calculation for the UAM project. Flight condition 1 set as
%cruise, flight condition 2 set as hover. Propeller angle of attack effect 
%on forces and moments calculated using ESDU 89047, assuming
%counter-rotating propellers

% Inputs:
%   TAS and Altitude: Flight condition evaluated
%   pars: geometry inputs to define airframe geometry
%   dir: structure with the name of the directorios. Must include .AVL
%   AerofoilDataset: Aerofoil aerodynamic characteristics dataset obtained with XFOIL 
%   BackProp: Response surface for the back propellers
%   sol_airframe: Initial estimation for the trimming point at cruise.
%   neural:Data for the neural network response surface
%   
% Outputs:
%   cost_funct: Cost functions, Power at cruise and power at hover
%   sol: trimming angles
%   Q: Torque
%   rps: rev/s
%   initialise: data to define aircraft
%   geo: Airframe geometry

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
prop = get_prop(pars)

%% Launch analysis and read data
try
    
    %% Checks for the constrains
    % Check propellers forces not be too high
    Taux = 9.81*2000*prop_cg_locback/(prop_cg_locback+prop_cg_loc(1))/2;
    T_backprop = 9.81*2000 - Taux*2;
    
    %Condition to check viability at TAS2 condition
    qbar2 = 0.5*cond.rho*TAS2^2;
    %Obtain spanwise location of mean aerodynamic chord
    y_mac = ymac(geo);
    % Obtain initial estimation for angle of attack and elevator deflection
    aoa_02 = ((2000*9.81/(qbar2*geo.S_w))-initialise.CL0)/initialise.CLa;
    if aoa_02>10*pi/180
        aoa_02=10*pi/180;
    end
    eta_02 = (-initialise.Cm0 - initialise.Cma*aoa_02)/initialise.CmdinA;
    %Trim at stall speed condition
    fun2 = @(x)trim_funct_airframe(x,initialise,geo,qbar2,CD,y_mac);
    x02 = [aoa_02,eta_02];
    %Options for the algorithm
    opts.FunctionTolerance = 50 ;
    opts.MaxIterations = 50000;
    opts.MaxFunctionEvaluations = 50000;
    opts.Diagnostics = 'off';
    opts.Display='off';
    %     opts.OptimalityTolerance = 5;
    [sol2] = fsolve(fun2,x02,opts);

    F2 = trim_funct_airframe(sol2,initialise,geo,qbar2,CD,y_mac);
    %Perform all the checks
    if (abs(sol2(1))>10.5*pi/180) || (abs(sol2(2))>35) || (abs(F2(1))>50) || (abs(F2(2))>50)
        cost_funct(1) = 9000000;
        cost_funct(2) = 10000000;
    else
        
        %% TRIM
        %% Operational point 1, cruise
        %Ranges in propeller runfile
        run.V = TAS;
        run.T = Treq;
        [~,~,~,~,~]=qprop_analysisv3...
            (prop,cond,dir,run,filename,AerofoilDataset);
        [Q,rps] = get_TQtables(run,filename,cond,prop);
        %Bear in mind that, following ESDU 89047 definition of n(rev/s),
        %CT=T/(rho*n^2*prop_diameter^4), CQ=Q/(rho*n^2*prop_diameter^5) and
        %CP=P/(rho*n^3*prop_diameter^5)
        %CT and CQ tables are defined as a function of V-rows rps-columns
        cost_funct(1) = Q * rps;
        %     fclose('all');
        delete(strcat(filename.qprop_results,'.txt'));
        %ESDU correction is applied to account for propeller angle of
        %attack
        T=Treq;
        J = TAS /(abs(rps)* prop.radius_tip*2);%Advance ratio
        CT = Treq/(cond.rho*rps^2*(prop.radius_tip*2)^4);%Thrust coefficient
        
        CZ_alpha = prop_correction(CT,prop.beta_root*prop.beta_rad2,J,prop.solidity);%Increment of normal force due to propeller angle of attack
        
        qbar = 0.5*cond.rho*TAS^2;
        q_prop = cond.rho*rps^2*(prop.radius_tip*2)^4*CZ_alpha; %
        % Forces(3) = cond.rho*rps^2*(prop.radius_tip*2)^4*CZ_alpha*alpha;
        y_mac = ymac(geo);
        %Trim the aircraft for the cruise condition
        fun = @(x)trim_funct(x,initialise,geo,T,qbar,q_prop,prop_cg_loc,CD,y_mac);
        x0 = [sol_airframe(1),-sol_airframe(1),sol_airframe(2)];
        
        opts = optimoptions('fsolve');
        opts = optimoptions(opts,'Algorithm','trust-region-dogleg');
        opts = optimoptions(opts,'FunctionTolerance' ,0.001);
        opts = optimoptions(opts,'MaxIterations',5000);
        opts = optimoptions(opts,'MaxFunctionEvaluations', 5000);
        opts = optimoptions(opts,'Diagnostics','off');
        opts = optimoptions(opts,'Display','off');
%         opts = optimoptions(opts,'OptimalityTolerance', 0.05);
        [sol,~,exitflag,~] = fsolve(fun,x0,opts);
        
        F = trim_funct(sol,initialise,geo,T,qbar,q_prop,prop_cg_loc,CD,y_mac);
        
        if (abs(sol(1))>4*pi/180) || (abs(sol(2))>6*pi/180)|| (abs(sol(3))>1.2) || (abs(F(1))>50) || (abs(F(2))>50) || (abs(F(3))>50)
            cost_funct(1) = 10000000;
            cost_funct(2) = 9000000;
            
        else
            
            %% 2nd cost function, Hover
            cond.rho=1.225;
            cond.mu=1.7894*10^-5;
            cond.a= 340.3;%flight_con{2}.a; %m/s speed of sound %316.4
            
            run2.V = 0;
            run2.T = 9.81*2000*prop_cg_locback/(prop_cg_locback+prop_cg_loc(1))/2;
            T_backprop = 9.81*2000 - run2.T*2;
            
            
            [~,~,~,~,~]=qprop_analysisv3...
                (prop,cond,dir,run2,filename,AerofoilDataset);
            [Q2,rps2] = get_TQtables(run2,filename,cond,prop);
            
            %Get optimized Back prop power
            Qrps3 = T_backprop^2*BackProp.coeff(1) + T_backprop*BackProp.coeff(2)...
                + BackProp.coeff(3);
            
            cost_funct(2) = Q2 * rps2*2 + Qrps3;
            % fclose('all');
            fclose('all');
            delete(strcat(filename.qprop_results,'.txt'));
        end
    end
catch
    cost_funct(1) = 20000000;
    cost_funct(2) = 20000000;
    fclose('all');
    if isfile(strcat(filename.qprop_results,'.txt'))
        delete(strcat(filename.qprop_results,'.txt'));
    end
end
end



function CZ_alpha = prop_correction(CT,pitch,J,sigma_e)
%Function to calculate the effect of the propeller angle of attack on the
%forces for counter-rotating propellers. Based on ESDU 89047.
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