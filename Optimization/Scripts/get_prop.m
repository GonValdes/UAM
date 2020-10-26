function [prop,prop_cg_loc,prop_cg_locback] = get_prop(pars,initialise)
% Obtain the propeller geometry necessary to run qprop

% initial set of parameters
%14-tip radius(m) [2-first y spline(%tip) 3-second y spline(%tip)]
%15-pitch root(deg) 16-pitch 1st y point(%root) 17-pitch 2nd y point(%root) 18-pitch root(%root)
%19-root chord(%tip radius) 20-chord 1st y point(%root) 21-chord 2nd y point(%root) 22-chord tip(%root)
%23-thickness root(t/c) 24-thickness y1(%root) 25-thickness y2(%root) 26-thickness tip(%root)
%27-camber root(%c) 28-camber y1(%root) 29-camber y2(%root) 30-camber tip(%root)
%31-TAS(m/s)  32-Thrust(N) 33-altitude(m)

%radius-root is 10% of tip
prop.radius_tip = pars(11);
prop.radius1 = 0.35;%root
prop.radius2 = 0.72;%root
%blade pitch angle
prop.beta_root = pars(12);%deg
prop.beta_rad1 = pars(13);%root
prop.beta_rad2 = pars(14);%root
prop.beta_tip = pars(15);%root
%chord
prop.cr = pars(16)*prop.radius_tip;%m
prop.chord_rad1 = pars(17);%root
prop.chord_rad2 = pars(18);%root
prop.ctaper = pars(19);%root
%aerofoil
%thickness
prop.afthickness_root = pars(20);%%t/c
prop.afthickness_rad1 = pars(21);%root
prop.afthickness_rad2 = pars(22);%root
prop.afthickness_tip = pars(23);%root
%camber
prop.afcamber_root = pars(24);%%/c
prop.afcamber_rad1 = pars(25);%root
prop.afcamber_rad2 = pars(26);%root
prop.afcamber_tip = pars(27);%root
%
% Treq = pars(28);
prop.cgforw =  pars(29);
prop.cgback =  pars(30);
%% Automatically defined or constant

prop.n_blades = 6;
prop.MaxCamberPos = 3;% Check where it is the value you want in the database
prop.ct = prop.ctaper*prop.cr; %m
prop.radius_root = 0.1*prop.radius_tip;%m
prop_cg_loc = [2; 1.5;0];%z positive downwards
prop_cg_loc(1) = initialise.CG - prop.cgforw;%forward propeller location respect to cg
prop_cg_locback = abs(initialise.CG - prop.cgback);%back propeller location respect to cg
prop.solidity = 4*prop.n_blades*mean([prop.cr,prop.cr*prop.chord_rad1,...
    prop.cr*prop.chord_rad2,prop.cr*prop.ctaper])/(3*pi*prop.radius_tip*2);% solidity
end