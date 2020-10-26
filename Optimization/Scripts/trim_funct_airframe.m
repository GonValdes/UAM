function F = trim_funct_airframe(vars,initialise,geo,qbar,CD,y_mac)
% Trim function for the airframe, using only elevator deflection and angle
% of attack
% -vars are elevator aoa(1) in rad and elevator deflection(2) in deg
% -qbar dynamic pressure
% -CD is a structure with the parasitic drag coefficients for the different
%  parts

% F(1)=Lift F(2)=pitch moment

F(1) = qbar*geo.S_w*(initialise.CL0 + initialise.CLa*vars(1) + initialise.CLdinA*vars(2))...
    -2000*9.81;%LIFT
F(2) = qbar*geo.S_w*geo.mac_w * (initialise.Cm0 + initialise.Cma*vars(1) + initialise.CmdinA*vars(2))+...
    qbar*geo.S_w*CD.c*(cos(vars(1))*geo.zle_c + sin(vars(1))*abs(initialise.CG-geo.mac_c/4))-...
    qbar*geo.S_w*CD.w*sin(vars(1))*abs(initialise.CG-(geo.xle_w+sind(geo.sweep1_w)*y_mac+geo.mac_w/4))+...
    qbar*geo.S_w*CD.fus* sin(vars(1))*abs(initialise.CG-geo.Cr_fus/4);%Moments
end