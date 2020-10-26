function [Forces,Power,e_p,TC] = prop_model(V,n,alpha,beta,density,Sn,Ap,pitch,...
                           J_Table,CP_Table,CT_Table,prop_type,sigma_e)
%prop_model: Propeller power and forces calculation in the spinner axis. 
% Method based on strip theory which was applicable to small or moderate
% inclination angles (approx. <20deg). The method also calculates the prop
% downwash caused by the slipstream.
% V:        Airspeed (m/s)
% n:        Rotor speed (rps) (<0 if prop rotates clockwise)
% alpha: 	Rotor angle of attack (rad)
% beta:     Rotor sideslip (rad)
% density:  Air density
%
% Sn:       Nacelle area
% Ap        prop disk area
% pitch:    Prop pitch angle referred to zero-lift line of section 0.75R (deg)
% J_Table:  Table of advance ratio
% CP_Table: Table of CP
% CT_Table: Table of CT
% prop_type: 1 single prop, 2 coaxial prop
% sigma_e:  Propeller solidity

if n ~= 0
    % advance ratio
    J = V /(abs(n)* D);
else
    J = 0;
end
% corrected advance ratio
% ESDU86017
Jeff = J * (1 - 0.255*Sn/Ap);

% propeller CP
CP = interp1(J_Table,CP_Table,Jeff,'linear','extrap',0);      
% propeller CT
CT = interp1(J_Table,CT_Table,Jeff,'linear','extrap',0);

if J ~= 0
    % ESDU 89047 
    Tc = CT * 8/(pi * J^2);
    kb = tan(pi/180*(pitch - 5))/(sigma_e * cos(pi/180*(pitch - 5)));
    if prop_type == 1
        CZ_alpha = -((4.25 * sigma_e)/(1 + 2 * sigma_e)) * sin(pi/180*(pitch + 3))*...
            (1 + ((3 * Tc/8)/sqrt(1 + (2*Tc/3))));  
        Cn_CZ = -(0.321/(sigma_e + tan(pi/180*(pitch + 5)))) * (1 + ((Tc * kb/3)...
            /(1 + (3 * Tc * sqrt(kb)/4))));
        CZ_alpha = CZ_alpha * (CT/Tc); 
        Cn_alpha = Cn_CZ * CZ_alpha;
    else
        CZ_alpha = -((3.86 * sigma_e)/(1 + sigma_e)) * sin(pi/180*(pitch + 9))*...
            (1 + ((3 * Tc/8)/sqrt(1 + (2*Tc/3))));
        CZ_alpha = CZ_alpha * (CT/Tc);   
        Cn_alpha = 0;
    end
else
    Tc=0;
    
    CZ_alpha =0;
    Cn_alpha =0;
end
Forces = zeros(6,1); 

%CT>0
% correction based on ESDU86017 for nacelle effect
Forces(1) = density*n^2*D^4*CT*(1 - 0.255*Sn/Ap);
% CY_beta sign of n (<0 if clockwise) 
% Forces(2) = -sign(n)*density*n^2*D^4*CZ_alpha*beta;
% CZ_apha < 0 
Forces(3) = density*n^2*D^4*CZ_alpha*alpha;
Forces(4) = density*n^2*D^5*CP/(2*pi);
% Cm_beta sign of n (<0 if clockwise) 
Forces(5) = sign(n)*density*n^2*D^5*Cn_alpha*beta;
% Cn_alpha - sign of n (>0 if clockwise) 
Forces(6) = - sign(n)*density*n^2*D^5*Cn_alpha*beta;
       
Power = density*n^3*D^5*CP;

% Downwash caused by propeller slipstream based on LabanOn-line Aircraft
% Aerodynamic Model Identification. PhD thesis, Proefschrift Technische
% Universiteit Delft, 1994.
e_p = (Forces(1)*sin(alpha)-Forces(3)*cos(alpha))/(density*V^2*Ap);
TC = Tc*pi/8;

end

