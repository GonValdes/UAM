function initialise = AVL_run(pars,flight_con,filename)

% File to obtain lift coefficiencts and aeroderivatives using AVL for the
% UAM project
% Inputs:
%   pars: parameters that can be modified. These parameters vary depending on
%       the configuration.
%   flight_con : Structure with the flight conditions. Must include
%       .Altitude, .TAS, .Mach, .rho
%   filename : Structure with the file names. Must include .geo in form of
%       string

%Outputs:
%   initialise: Structure with aeroderivatives

%% Define Parameters

%lengths in m, angles in rad

%cr-root chord,ct-root tip,b-semispan,S-surface,mac-mean aerodynamic chord
%taper- taper ratio,xle-xposition leading edge,yle-yposition leading edge
%zle-zposition leading edge,aoar-angle of attack root
%aoat-angle of attack tip
%inner_ail-Elevator
%outer_ail-Aileron

% Suffix --> _w:wing, _c:canard, _f:fin, _fus:fuselage

%% Parameters definition

%Parameters to be changed
geo.Cr_w = pars(1);%(m)
geo.Ct_w = pars(2);
geo.sweep1_w = pars(3);%(deg)
geo.aoar_w = pars(4);%(deg)
geo.airfoilroot_w = '3312';
geo.airfoiltip_w = '3312';
geo.xle_w = pars(5);%(m)
%canard
geo.Cr_c = pars(6);%(m)
geo.b_c = pars(7);%(m)
geo.aoar_c = pars(8);%(deg)
geo.zle_c = pars(9);%(m)
%CG position
initialise.CG = pars(10);
%Fin
geo.b_f= 0.8;%(m)

% FIXED parameters
geo.b_w= 4.1;
geo.twist_w = 0;%(deg);
geo.yle_w = 0.9;
geo.zle_w = 0;
%Control surface parameters
%Canard elevator
geo.innery_innerail = 10;%Start of the elevator(%span)
geo.outery_innerail = 90;%End of the elevator (%span)
geo.x_innerail = 60;%Percentage of the chord of elevator(%chord)
%Wing aileron
geo.innery_outerail = 60;%Start of the aileron(%span)
geo.outery_outerail = 90;%End of the aileron (%span)
geo.x_outerail = 75;%Percentage of the chord of aileron(%chord)
%Canard
geo.Ct_c = geo.Cr_c;%(m)
geo.sweep1_c = 0;%(deg)
geo.twist_c = 0;%(deg)
geo.xle_c = 0;%(m)
geo.yle_c = 0.9;%(m)
geo.airfoilroot_c= '3412'; %airfoil used
geo.airfoiltip_c = '3412';
%Fin 
geo.sweep1_f = 15;%(deg)
geo.Cr_f = 0.5;%(m)
geo.xle_f = 9-geo.Cr_f;%(m)
geo.yle_f = 0.8;%(m)
geo.zle_f = 0.3;%(m)
geo.airfoil_f = '0012';

%CALCULATED
geo.aoat_w = geo.aoar_w-geo.twist_w;
geo.S_w = (geo.Cr_w+geo.Ct_w)/2*geo.b_w*2;
geo.taper_w = geo.Ct_w/geo.Cr_w;
geo.mac_w= geo.Cr_w *2/3 *(1+geo.taper_w+geo.taper_w^2)/(1+geo.taper_w);
geo.meanc_w = (geo.Cr_w +geo.Ct_w)/2;
%canard
% geo.Ct_c = geo.Cr_c-geo.b_c*tan(geo.sweep1_c*pi/180);
geo.aoat_c = geo.aoar_c-geo.twist_c;
geo.S_c = (geo.Cr_c+geo.Ct_c)/2*geo.b_c*2;
geo.taper_c = geo.Ct_c/geo.Cr_c;
geo.mac_c = geo.Cr_c *2/3 *(1+geo.taper_c+geo.taper_c^2)/(1+geo.taper_c);
%fin
geo.Ct_f = geo.Cr_f-geo.b_f*tan(geo.sweep1_f*pi/180);
geo.S_f = (geo.Cr_f+geo.Ct_f)/2*geo.b_f;
geo.taper_f = geo.Ct_f/geo.Cr_f;
geo.mac_f= geo.Cr_f *2/3 *(1+geo.taper_f+geo.taper_f^2)/(1+geo.taper_f);


%FUSELAGE, always fixed
geo.Cr_fus = 7;%(m)
geo.b_fus = 0.9;%the semi span
geo.airfoil_fus = '4421';%0421MOD this was cool also
geo.S_fus = geo.Cr_fus*geo.b_fus*2;

geo.aspect_ratio_w = ((geo.b_w+geo.b_fus)*2)^2/(geo.S_w);

%% Oswald factor definition. From Estimating the Oswald Factor... paper
geo.sweep25_w = abs(atan((geo.b_w*tan(geo.sweep1_w*pi/180)+geo.Ct_w/4-geo.Cr_w/4)/geo.b_w)*180/pi);
f1 = geo.taper_w - (-0.357 + 0.45*exp(0.0375*geo.sweep1_w));%(38)
f2 = 0.0524*f1^4 - 0.15*f1^3 + 0.1659*f1^2 - 0.0706*f1 + 0.0119;
e_w = 1/(1 + f2*geo.aspect_ratio_w);
e_fus = 1-2*(geo.b_fus/(geo.b_w+geo.b_fus))^2;
e_zld = 0.804;%zero-lift drag correction

geo.eOswald =e_w*e_fus*e_zld;
geo.K = 1/(pi*geo.aspect_ratio_w*geo.eOswald);

%% Define Inertia properties
initialise.mass = 2000;%mass fixed at 2000kg

InertiaMatrix = GetInertiaMatrix(geo,initialise.CG);

initialise.Ixx = InertiaMatrix(1,1);
initialise.Iyy = InertiaMatrix(2,2);
initialise.Izz = InertiaMatrix(3,3);
initialise.Ixz = InertiaMatrix(1,3);

%% Geometry file creation
%Here, you create the avl file
geo_defv3(filename.geo,geo,initialise);

    %% Write AVL case files and collect data
    try
    % counter = 0;
    runfilename = ['UAM_h' num2str(flight_con.Altitude) '_TAS' ...
        num2str(flight_con.TAS) '_CG' num2str(initialise.CG)];
    
    generate_AVLcase(runfilename,flight_con,...
        initialise,initialise.mass,initialise.CG);
    
    [ST_Data] = get3DAero(filename.geo,runfilename);
    
    if isfile(strcat(filename.geo,'.st'))
        delete(strcat(filename.geo,'.st'));
    end

    if isfile(strcat(filename.geo,'.avls'))
        delete(strcat(filename.geo,'.avls'));
    end
    if isfile(strcat(runfilename,'.run'))
        delete(strcat(runfilename,'.run'));
    end
    
    %Forces
    initialise.CL0 = ST_Data.CL;
    initialise.CLa = ST_Data.CLa;%rad-1
    initialise.CLq = ST_Data.CLq;
    
    initialise.CDind = ST_Data.CDind;
    initialise.Kind = ST_Data.CDind/ST_Data.CL^2;
    initialise.CDff = ST_Data.CDff;
    initialise.Kff = ST_Data.CDff/ST_Data.CL^2;
    
    initialise.CYb = ST_Data.CYb;
    initialise.CYp = ST_Data.CYp;
    initialise.CYr = ST_Data.CYr;
    
    %                 %Moments
    initialise.Cm0 = ST_Data.Cm;
    initialise.Cma = ST_Data.Cma;
    initialise.Cmq = ST_Data.Cmq;
    %
    initialise.Clb = ST_Data.Clb;
    initialise.Clp = ST_Data.Clp;
    initialise.Clr = ST_Data.Clr;
    
    initialise.Cnb = ST_Data.Cnb;
    initialise.Cnp = ST_Data.Cnp;
    initialise.Cnr = ST_Data.Cnr;
    
    %inner_aileron
    initialise.CLdinA = ST_Data.CLdinA;
    initialise.CmdinA = ST_Data.CmdinA;
    initialise.CDdinA = abs(ST_Data.CDdinA);
    %outer_aileron
    initialise.CldoutA = ST_Data.CldoutA ;
    initialise.CYdoutA = ST_Data.CYdoutA ;
    initialise.CndoutA = ST_Data.CndoutA ;
    initialise.CDdoutA = abs(ST_Data.CDdoutA); 
    %Right rudder
    initialise.CldRr  = ST_Data.CldRr ;
    initialise.CYdRr = ST_Data.CYdRr;
    initialise.CndRr = ST_Data.CndRr;
    initialise.CDdRr = abs(ST_Data.CDdRr);
%     %Left rudder
%     initialise.CldRl = ST_Data.CldRl;
%     initialise.CYdRl = ST_Data.CYdRl ;
%     initialise.CndRl = ST_Data.CndRl ;
%     initialise.CDdRl = ST_Data.CDdRl;

    
    catch
    
    %Forces
    initialise.CL0 = NaN;
    initialise.CLa = NaN;
    initialise.CLq = NaN;
    
%     initialise.CD0_AVL = NaN;
    initialise.CDind = NaN;
    initialise.CDff = NaN;
    %
    initialise.CYb = NaN;
    initialise.CYp = NaN;
    initialise.CYr = NaN;
    
    %                 %Moments
    initialise.Cm0 = NaN;
    initialise.Cma = NaN;
    initialise.Cmq = NaN;
    %
    initialise.Clb = NaN;
    initialise.Clp = NaN;
    initialise.Clr = NaN;
    
    initialise.Cnb = NaN;
    initialise.Cnp = NaN;
    initialise.Cnr = NaN;
    
    %inner_aileron
    initialise.CLdinA = NaN;
    initialise.CmdinA = NaN;
    initialise.CDdinA = NaN;
    %outer_aileron
    initialise.CldoutA = NaN;
    initialise.CYdoutA = NaN;
    initialise.CndoutA = NaN ;
    initialise.CDdoutA = NaN; 
    %Right rudder
    initialise.CldRr  = NaN;
    initialise.CYdRr =NaN;
    initialise.CndRr = NaN;
    initialise.CDdRr = NaN;
    
    
    if isfile(strcat(filename.geo,'.st'))
        delete(strcat(filename.geo,'.st'));
    end
    if isfile(strcat(filename.geo,'.avls'))
        delete(strcat(filename.geo,'.avls'));
    end
    if isfile(strcat(runfilename,'.run'))
        delete(strcat(runfilename,'.run'));
    end
   
end
end
