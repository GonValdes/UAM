function [cost_funct] = Cost_function_optiv4neural_onlyProp_noTrim(TAS,Altitude,pars,airframe,dir,filename,AerofoilDataset,BackProp,Treq)
% Cost function considering only the propellers. No ESDU correction. Power
% evaluated at hover and cruise
%
% Inputs:
%   TAS and Altitude: Flight condition evaluated
%   pars: geometry inputs to define airframe geometry
%   dir: structure with the name of the directorios. Must include .AVL
%   AerofoilDataset: Aerofoil aerodynamic characteristics dataset obtained with XFOIL 
%   BackProp: Response surface for the back propellers
%   Treq: Thrust required at cruise= Drag obtained before, considering only
%         the airframe
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


%% Propeller Block
%Obtain forward propeller data
prop = get_prop(pars)

%% Launch analysis and read data
try
    
    %% Check propellers forces distribution between back and forward propellers
    
    Taux = 9.81*2000*prop_cg_locback/(prop_cg_locback+prop_cg_loc(1))/2;
    T_backprop = 9.81*2000 - Taux*2;
    if T_backprop > 0.65*2000*9.81
        cost_funct(1) = 9000000;
        cost_funct(2) = 10000000;
    else
        %% TRIM
        %Flight condition 1
        %Ranges in propeller runfile
        run.V = TAS;
        run.T = Treq;
        %Run QPROP and read data
        [~,~,~,~,~]=qprop_analysisv3...
            (prop,cond,dir,run,filename,AerofoilDataset);
        [Q,rps] = get_TQtables(run,filename,cond,prop);
        %Bear in mind that, following ESDU 89047 definition of n(rev/s),
        %CT=T/(rho*n^2*prop_diameter^4), CQ=Q/(rho*n^2*prop_diameter^5) and
        %CP=P/(rho*n^3*prop_diameter^5)
        %CT and CQ tables are defined as a function of V-rows rps-columns
        
        %Obtain the cost function value
        cost_funct(1) = Q * rps;
        %     fclose('all');
        delete(strcat(filename.qprop_results,'.txt'));
        
        
        
        %% 2nd cost function, Hover
        %Define second flight condition, hover
        cond.rho=1.225;
        cond.mu=1.7894*10^-5;
        cond.a= 340.3;%flight_con{2}.a; %m/s speed of sound %316.4
        
        run2.V = 0;
        run2.T = 9.81*2000*prop_cg_locback/(prop_cg_locback+prop_cg_loc(1))/2;
        T_backprop = 9.81*2000 - run2.T*2;
        
        %Run QPROP and read data
        [~,~,~,~,~]=qprop_analysisv3...
            (prop,cond,dir,run2,filename,AerofoilDataset);
        [Q2,rps2] = get_TQtables(run2,filename,cond,prop);
        
        %Get optimized Back prop power with response surface
        Qrps3 = T_backprop^2*BackProp.coeff(1) + T_backprop*BackProp.coeff(2)...
            + BackProp.coeff(3);
        
        %Get second cost function, power at hover
        cost_funct(2) = Q2 * rps2*2 + Qrps3;
        % fclose('all');
        fclose('all');
        delete(strcat(filename.qprop_results,'.txt'));
        
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