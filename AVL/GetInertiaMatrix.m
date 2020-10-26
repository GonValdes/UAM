function InertiaMatrix = GetInertiaMatrix(geo,CG)
% Calculate inertia for fuselage, fin, canard and wing for the UAM project
% From ASD-TR-79-5004. Inertia calculation procedure for preliminary
% design. Charles Lanham

%Inputs:

%---geo:
%Everything stated in AVL axes (x positive going rearwards starting
%from nose, y -right wing, z-up)
%Output: inertia matrix in the body axes from the cg: positives: x- forward, y- right z-down
%---CG:
%Include the cg position in AVL axes(x positive going rearwards starting
%from nose, y -right wing, z-up)
%

%M- mass of the correspondent surface (kg)
% b- span of surface from root to tip (m)
% t_r - thickness at root (m)
% t_t - thickness at tip (m)
% c - choord at root (m)
% sweep_t - sweep trailing edge (rad)
% sweep_l - sweep leading edge(rad)

%surface - 1:wing 2:canard or tails 3:fuselage

% Mn mass of nose (kg)
% ln lenght nose(m)
% Mc mass of center(kg)
% lc lenght center(m)
% Average fuselage radius (m)

% Created by: Gonzalo Valdes Apr 2020

%% Definition of the mass of each component. Selected by estimations based on
%other aircraft.

geo.M_w = 30+247;%kg 
geo.M_c = 79+10; 
geo.M_f = 8+10;%Fin
geo.M_fus = 466+150;
geo.CGx = CG;
geo.CGy = 0;
geo.CGz = 0;

CGbattery = [5 0 0];
Mbat=400;

Mpax = 100;
CGpax1 = [2.1 0 0];
CGpax2 = [3 0.3 0];
CGpax3 = [3 -0.3 0];

%% Obtain inertia matrix about each component centroid

%Wing
params.M = geo.M_w;
params.b = geo.b_w;
params.t_r = str2double(geo.airfoilroot_w(3:4));
params.t_t = str2double(geo.airfoiltip_w(3:4));
params.c = geo.Cr_w;
params.sweep_t = geo.sweep1_w;
params.sweep_l = geo.Cr_w -(geo.Ct_w+geo.b_w*sin(geo.sweep1_w));

[inertia_w]=inertia_calc(params,1);

%Canard
params.M = geo.M_c;
params.b = geo.b_c;
params.t_r = str2double(geo.airfoilroot_c(3:4));
params.t_t = str2double(geo.airfoilroot_c(3:4));
params.c = geo.Cr_c;
params.sweep_t = geo.sweep1_c;
params.sweep_l = geo.Cr_c -(geo.Ct_c+geo.b_c*sin(geo.sweep1_c));

[inertia_c]=inertia_calc(params,2);

%Fin
params.M = geo.M_f;
params.b = geo.b_f;
params.t_r = str2double(geo.airfoil_f(3:4)) ;
params.t_t = str2double(geo.airfoil_f(3:4)) ;
params.c = geo.Cr_f;
params.sweep_t = geo.sweep1_f;
params.sweep_l = geo.Cr_f -(geo.Ct_f+geo.b_f*sin(geo.sweep1_f));

[inertia_fin]=inertia_calc(params,2);

%Convert the inertia matrix as the fin is vertical
axis_change = [inertia_fin.Iyy,inertia_fin.Izz,inertia_fin.Ycentroid];
inertia_fin.Iyy = axis_change(2);
inertia_fin.Izz = axis_change(1);
inertia_fin.Zcentroid = axis_change(3);
inertia_fin.Ycentroid = 0;


%fuselage
params.ln = 3;
params.lc = 4;
params.Wn = geo.M_fus*0.3;
params.Wc = geo.M_fus*0.7;
params.R = 0.8;
params.zfus = 0;
params.xfus = 3.5; 
[inertia_fus]=inertia_calc(params,3);



%% Obtain the Inertia matrices about the CG
%Wing
dist_w1 = [inertia_w.Xcentroid + geo.xle_w - geo.CGx,inertia_w.Ycentroid+geo.yle_w,...
    inertia_w.Zcentroid+geo.zle_w];
dist_w2 = [inertia_w.Xcentroid+geo.xle_w-geo.CGx,-inertia_w.Ycentroid-geo.yle_w,...
    inertia_w.Zcentroid+geo.zle_w];
inertia_matrixCG_w1 = GetInertiaCG(inertia_w,dist_w1,geo.M_w);
inertia_matrixCG_w2 = GetInertiaCG(inertia_w,dist_w2,geo.M_w);

%Canard
dist_c1 = [inertia_c.Xcentroid+geo.xle_c-geo.CGx,inertia_c.Ycentroid+geo.yle_c,...
    inertia_c.Zcentroid+geo.zle_c];
dist_c2 = [inertia_c.Xcentroid+geo.xle_c-geo.CGx,-inertia_c.Ycentroid-geo.yle_c,...
    inertia_c.Zcentroid+geo.zle_c];
inertia_matrixCG_c1 = GetInertiaCG(inertia_c,dist_c1,geo.M_c);
inertia_matrixCG_c2 = GetInertiaCG(inertia_c,dist_c2,geo.M_c);

%Fin
dist_fin1 = [inertia_fin.Xcentroid+geo.xle_f-geo.CGx,inertia_fin.Ycentroid+geo.yle_f,...
    inertia_fin.Zcentroid+geo.zle_f];
inertia_matrixCG_fin1 = GetInertiaCG(inertia_fin,dist_fin1,geo.M_f);

dist_fin2 = [inertia_fin.Xcentroid+geo.xle_f-geo.CGx,-inertia_fin.Ycentroid-geo.yle_f,...
    inertia_fin.Zcentroid+geo.zle_f];
inertia_matrixCG_fin2 = GetInertiaCG(inertia_fin,dist_fin2,geo.M_f);

%Fus
dist_fus = [inertia_fus.Xcentroid-geo.CGx,inertia_fus.Ycentroid,inertia_fus.Zcentroid];

inertia_matrixCG_fus = GetInertiaCG(inertia_fus,dist_fus,geo.M_fus);


%% Point masses
%Systems, batteries, pax
inertiapoint.Ixx=0;inertiapoint.Iyy=0;inertiapoint.Izz=0;inertiapoint.Ixy=0;inertiapoint.Ixz=0;inertiapoint.Iyz=0;

%battery
M=Mbat;
dist_bat = [CGbattery(1)-geo.CGx,CGbattery(2)-geo.CGy,CGbattery(3)-geo.CGz];
inertia_matrixCG_bat = GetInertiaCG(inertiapoint,dist_bat,M);

%passengers
M=Mpax;

dist_pax1 = [CGpax1(1)-geo.CGx,CGpax1(2)-geo.CGy,CGpax1(3)-geo.CGz];
dist_pax2 = [CGpax2(1)-geo.CGx,CGpax2(2)-geo.CGy,CGpax2(3)-geo.CGz];
dist_pax3 = [CGpax3(1)-geo.CGx,CGpax3(2)-geo.CGy,CGpax3(3)-geo.CGz];

inertia_matrixCG_pax1 = GetInertiaCG(inertiapoint,dist_pax1,M);
inertia_matrixCG_pax2 = GetInertiaCG(inertiapoint,dist_pax2,M);
inertia_matrixCG_pax3 = GetInertiaCG(inertiapoint,dist_pax3,M);

%% Final sum of every component

InertiaMatrix=inertia_matrixCG_w1 + inertia_matrixCG_w2 + inertia_matrixCG_c1...
    + inertia_matrixCG_c2 + inertia_matrixCG_fin1 + inertia_matrixCG_fin2 +...
    inertia_matrixCG_fus+inertia_matrixCG_bat + ...
    inertia_matrixCG_pax1 + inertia_matrixCG_pax2+inertia_matrixCG_pax3;
%Convert to x positive forward and z positive downward:
%changing all the coupled x and z to neg, while also
%taking into account that the non-diagonal terms in the final matrix are
%negative:
%I = [Ixx -Ixy -Ixz ,...]
InertiaMatrix = [abs(InertiaMatrix(1,1)) InertiaMatrix(1,2) -InertiaMatrix(1,3);
    InertiaMatrix(2,1) abs(InertiaMatrix(2,2)) InertiaMatrix(2,3);
    -InertiaMatrix(3,1) InertiaMatrix(3,2) abs(InertiaMatrix(3,3))]; 




end

%% Other functions
function inertia_matrixCG = GetInertiaCG(inertia,dist,M)
%Transform the inertia matrix of a certain component from its own centroid
%to the CG
%Inputs:
%   -inertia: structure with all the inertia properties about the original
%   point
%   -dist:distance to the original point from the CG
%   -M: mass of the component

inertia_matrix = [inertia.Ixx inertia.Ixy inertia.Ixz;
    inertia.Ixy inertia.Iyy inertia.Iyz;
    inertia.Ixz inertia.Iyz inertia.Izz];

dist_matrix = M*[dist(2)^2+dist(3)^2 dist(1)*dist(2) dist(1)*dist(3);
    dist(1)*dist(2) dist(1)^2+dist(3)^2 dist(2)*dist(3);
    dist(1)*dist(3) dist(2)*dist(3) dist(1)^2+dist(2)^2];

inertia_matrixCG = inertia_matrix + dist_matrix;

end

function [inertia]=inertia_calc(geo,surface_type)

%Inertia calculation of every component, from ASD-TR-79-5004

%Inputs inside geo:

%M- mass of the correspondent surface (kg)
% b- span of surface from root to tip (m)
% t_r - thickness at root (m)
% t_t - thickness at tip (m)
% c - choord at root (m)
% sweep_t - sweep trailing edge (rad)
% sweep_l - sweep leading edge(rad)

%surface - 1:wing 2:canard or tails 3:fuselage

% Mn mass of nose (kg)
% ln lenght nose(m)
% Mc mass of center(kg)
% lc lenght center(m)
% Average fuselage radius (m)

% Created by: Gonzalo Valdes Apr 2020

if (surface_type == 1) || (surface_type == 2)
    M =geo.M;
    b = geo.b;
    t_r = geo.t_r;
    t_t = geo.t_t;
    sweep_l = geo.sweep_l;
    sweep_t = geo.sweep_t;
    c = geo.c;
   
    V = b*(t_r*(c + b/2*(tan(sweep_t)-tan(sweep_l))) - ...
        (t_r-t_t)*(c/2 + b/3*(tan(sweep_t)-tan(sweep_l)))); %m^3
    
    Ixx = abs(M*b^3/V * ((t_r-t_t)*(c/4 + b*tan(sweep_t)/5 - b*tan(sweep_l)/5) + ...
        t_r*(c/3 + b*tan(sweep_t)/4 - b*tan(sweep_l)/4))); % kg*m^2
    
    Iyy =  abs(M*b/V * (t_r*(c^3/3 + b*c*tan(sweep_t)*(c/2 + b*tan(sweep_t)/3)+ ...
        b^3/12*(tan(sweep_t)^3-tan(sweep_l)^3))...
        - ((t_r-t_t)*(c^3/6 + b*c*tan(sweep_t)*(c/3 + b*tan(sweep_t)/4) ...
        + b^3/15*(tan(sweep_t)^3-tan(sweep_l)^3)))));
    
    Izz = Ixx+Iyy;
    
    if surface_type==1
        %wing
        k0=0.703;
    elseif surface_type==2
        %horizontal tail, canard or vertical tail
        k0 = 0.771;
    end
    
    aux = sort([c,b*tan(sweep_l),b*tan(sweep_l)+c],'ascend');
    
    c_a= aux(1)*39.37;
    c_b= aux(2)*39.37;
    c_c= aux(3)*39.37;
    
    inertia.Xcentroid = (-c_a^2+c_b^2+c_c*c_b+c_c^2)/(3*(c_b+c_c-c_a))*((k0)^0.5)/39.37;
    inertia.Ycentroid = abs(b^2/V *(t_r*(c/2 + b/3*(tan(sweep_t)-tan(sweep_l)))- ...
        (t_r-t_t)*(c/3+b/4*(tan(sweep_t)-tan(sweep_l)))));
    inertia.Zcentroid=0;
    Ixz=0;
end


if surface_type == 3
    ln = geo.ln;
    lc = geo.lc;
    Wn = geo.Wn;
    Wc = geo.Wc;
    R = geo.R;
    zb = geo.zfus;
    
    Ixx = 0;%R^2/2*(Wn+2*Wc) + (Wn+Wc)*zb^2;
    Iyy = R^2/4 *(Wn+2*Wc) + ln^2*(Wn/2+Wc) + lc^2*(Wc/3) + lc*ln*Wc +2/3*(Wn+Wc)*zb^2;
    Izz = Iyy - (Wn+Wc)*zb^2;
    Ixz = 0;%Wn *3/4*ln*zb + Wc*zb*(ln+lc/2);
    
    inertia.Xcentroid = geo.xfus;
    inertia.Ycentroid = 0;
    inertia.Zcentroid = geo.zfus;
end

inertia.Ixx = Ixx;
inertia.Iyy = Iyy;
inertia.Izz = Izz;
inertia.Ixy = 0;
inertia.Ixz = Ixz;
inertia.Iyz = 0;
end

