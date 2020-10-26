function [a320_UC] = A320_UC(maingear_position,nosegear_position)
%% Undercarriage drag prediction
%
%   All parameters in metric units unless stated otherwise
%
%   Julien Enconniere       11/12/2019

% Based on ESDU 79015 - Undercarriage drag prediction methods


%% Main langing gear characteristics.

maingear.Struts.dZsection = [0.500 0.362 0.500 0.906];
maingear.Struts.diameter =  [0.224	0       0       0;...
                    0.224   0.090   0       0;...
                    0.224   0.090   0.112   0;...
                    0.224   0.090   0.135   0.067];
                
maingear.Struts.aspect =    [1.00	0       0       0;...
                    1.00   1.5      0       0;...
                    1.00   1.5     1.1      0;...
                    1.00   0.95    1.1    1.5];
              
maingear.Struts.phi =       [-8.0	0      0       0;...
                    -8.0   -39     0       0;...
                    -8.0   -39    -32      0;...
                    -8.0   -39    -32     0.0];
                    
maingear.Struts.omega =     [0.0    0      0       0;...
                    0.0   38      0       0;...
                    0.0   38    0.0       0;...
                    0.0   38    0.0     55];
                
maingear.Struts.door.td = [0.05  0; 0.05  0; 0.05  0; 0.05  0];  
                         
maingear.Struts.door.ld = [1.0  0;  2.0  0;  2.0  0;  2.0  0];
                             
maingear.Struts.door.dd = [-0.04  0;  -0.04  0;  -0.04  0;  -0.04  0];

maingear.Wheel.dt = 1.17;
maingear.Wheel.b = 1.35;
maingear.Wheel.m = 0.54;
maingear.Wheel.n = 0.5;

maingear.cw = 4.822; % wing chord at the landing gear
maingear.tw = maingear.cw*0.152; % wing thickness at the landing gear
maingear.cfw = 0.22; % flap to wing chord ratio 

%% Nose landing gear characteristics 
nosegear.Struts.dZsection = [0.612 0.450 0.467];
nosegear.Struts.diameter =   [0.17  0     0    0;...
                     0.195 0.056 0.056 0.056;...
                     0.195 0.056 0.056 0.056];
nosegear.Struts.aspect =[1  0   0   0;...
                 1  1   2   2;...
                 1  1   2   2];             
nosegear.Struts.phi = [ 9.0  0   0   0;...
                9.0 -33  9.5  9.5;...
                9.0 -33  9.5  9.5];
nosegear.Struts.omega =[0   0   0   0;...
                0   0  33   -33;...
                0   0  33   -33];
nosegear.Struts.door.td = [0  0; 0  0; 0.05  0.05];
nosegear.Struts.door.ld = [0  0; 0  0;  0.5  0.5];
nosegear.Struts.door.dd = [0  0; 0  0;  0.380 -0.380];         

nosegear.Wheel.dt = 0.78;
nosegear.Wheel.b = 0.72;
nosegear.Wheel.m = 0.25;
nosegear.Wheel.n = 0.33;

%% Output tables
a320_UC.re = [3e5 4.26e5 4.28e5 6.40e5 6.42e5 7e5];
a320_UC.alpha = [-4 -2.5 0 2.5 5 7.5 10 12.5];
a320_UC.dflap = [0 5 10 15 20 25 30 35 40];

a320_UC.CD_maingear = zeros(length(a320_UC.re),length(a320_UC.alpha), length(a320_UC.dflap));
a320_UC.z_maingear = zeros(length(a320_UC.re),length(a320_UC.alpha), length(a320_UC.dflap));

a320_UC.CD_nosegear = zeros(length(a320_UC.re),length(a320_UC.alpha));
a320_UC.z_nosegear = zeros(length(a320_UC.re),length(a320_UC.alpha));

% Landing gear ground 3.2m below center of gravity
for i = 1:length(a320_UC.re)
   for j = 1:length(a320_UC.alpha)
       for k = 1:length(a320_UC.dflap)
           [a320_UC.CD_maingear(i,j,k),a320_UC.z_maingear(i,j,k)] = CD_LandingGear(...
               maingear.Struts, maingear.Wheel, a320_UC.alpha(j),...
               maingear.cw, maingear.tw, maingear.cfw, a320_UC.dflap(k),...
               a320_UC.re(i));
       end
       
       [a320_UC.CD_nosegear(i,j),a320_UC.z_nosegear(i,j)] = CD_LandingGear(...
           nosegear.Struts, nosegear.Wheel, a320_UC.alpha(j), 0, 0, 0, 0,...
           a320_UC.re(i));
   end 
end

% flap deflection has not impact of z_main... reduction of the size of
% matrix
a320_UC.z_maingear = a320_UC.z_maingear(:,:,1);

% change of referential
a320_UC.z_maingear = maingear_position - a320_UC.z_maingear;
a320_UC.z_nosegear = nosegear_position - a320_UC.z_nosegear;




end

function [CD_total,z_aero] = CD_LandingGear(Struts, Wheel, alpha,...
    cw, tw, c_f2w, delta_F,Re)
% CD prediction of an entire landing gear - The function divides the 
% landing gear in a number of sections. Section 0 corresponds to the wheel. 
% The following sections account for the complexity of the landing gear.
% The required inputs are defined as followed 
% Struts:   Define the struts architecture, door position relative to
%   main strut vertical axis. Includes;
%               dZsection:     Sections length - Dim n (n = number of
%                   sections)
%               diameter:      Diameter of each strut in each section - 
%                   Dim nxm (m = max number of strut)        
%               aspect:        Aspect ration of each strut. Value of 1 if
%                   strut with circular section. Dim nxm
%               phi:           Inclination of strut in each section measured 
%                   from plane normal to freestream direction - Dim nxm       
%               omega:         Angle between strut and plane normal to wheel
%                   axle - Dim nxm
%               door.td:       Door thickness - Dim nxl
%               door.ld:       Door length in the direction of the
%                   freestream - Dim nxl
%               door.dd:        Door position from plane normal to wheel
%                   axle - Dim nxl
% Wheel:    Wheel geometry
%               dt:            Total depth of wheel combination normal to
%                   freestream direction 
%               b:             Width of wheel or maximum width of wheel
%                   combination
%               m:             Depth of gap between inner wheels of 
%                   multi-wheel configuration 
%               n:             Width of gap between inner wheels of
%                   multi-wheel configuration
%              
% alpha:    Aircraft incidence
% cw:       Local wing chord 
% tw:       Local wing thickness
% cfw;      Flap to wing chord ratio
% delta_F:  Falp deflection
% Re:       Reynolds number 
%
% Ex:         ^ z     
%      _______|_____
%      \\  //| | ||         Section 3
%       \\// | | ||    - - - - - -
%        \\  | | ||         Section 2
%         \\ | | ||
%          \\| | ||    - - - - - - -
%            | | ||         Section 1
%            | | || 
%        --- | | ---   - - - - - - -
%    <--|   || ||   |       Section 0 - Wheel
%    y  |   | - |   | 
%        ---     --- 
% Assume no effect of door on wheel

load('landing_gear.mat');

% effect of incidence
Struts.phi = Struts.phi + alpha;

% Wheels drag 
%Re = V*rho/visc; % /m

CD_w = CD_wi(Wheel.dt,Wheel.b,90,Wheel.m,Wheel.n,Re,CD_wheel1,CD_wheel2);
% Drag of single (or multi) wheel with strut effect
% dt:       Total deth of wheel combination normal to freestream direction
% Diameter of wheel
% b:        Width of wheel or maximum width of wheel combination
% Theta:    Angle between strut and plane normal to wheel axle
% m:        Depth of gap between inner wheels of multi-wheel conf
% n:        Width of gap between inner wheels of multi-wheel conf
% Re_m:     Reynolds number


% Aero moment at the landing gear wheel
M_gear = 0;

% Initialisation
CD_s = zeros(size(Struts.diameter));
Struts_dy = zeros(size(Struts.diameter));
Struts_dx = zeros(size(Struts.diameter));
Struts_y = zeros(size(Struts.diameter));
Struts_x = zeros(size(Struts.diameter));
CD_d =  zeros(size(Struts.door.dd));
Zsection = zeros(size(Struts.dZsection));
L_door = zeros(size(Struts.door.dd,2));

for i = 1:length(Struts.dZsection) % nb of sections, ie number of rows
    for j = 1:length(Struts.diameter(i,:)) % number of struts, ie number of columns
        if Struts.diameter(i,j) ~= 0 % strut exists in the section i
            if Struts.aspect(i,j) == 1 % Assume circular section strut
                % include effect of inclinations
                CD_s(i,j) = CD_si_circ(...
                    Struts.dZsection(i)*cosd(Struts.phi(i,j))/cosd(Struts.omega(i,j)),...
                    Struts.diameter(i,j),...
                    Struts.phi(i,j));
            else % Rectangular section strut
                % for the specific case of square strut an aspect ratio of
                % 0.99 will have to be set 
                % include effect of inclinations
                CD_s(i,j) = CD_si_rect(...
                    Struts.dZsection(i)*cosd(Struts.phi(i,j))/cosd(Struts.omega(i,j)),...
                    Struts.diameter(i,j)*Struts.aspect(i,j),...
                    Struts.diameter(i,j),...
                    Struts.phi(i,j),...
                    CD_strut);
            end
             
            % Avg position of each strut in each section
            Struts_dy(i,j)= 0.5*Struts.dZsection(i)*tand(Struts.omega(i,j)); 
            Struts_dx(i,j)= 0.5*Struts.dZsection(i)*tand(Struts.phi(i,j));
           
            Struts_y(i,j) = 0;
            Struts_x(i,j) = 0;
            if i ~= 1
                for k = 1:i-1
                    Struts_y(i,j) = Struts_y(i,j) + 2*Struts_dy(k,j);
                    Struts_x(i,j) = Struts_x(i,j) + 2*Struts_dx(k,j);
                end              
            end
            Struts_y(i,j) = Struts_y(i,j) + Struts_dy(i,j);
            Struts_x(i,j) = Struts_x(i,j) + Struts_dx(i,j);
            
            for k = 1:length(Struts.door.dd(i)) % number of doors, ie number of columns
                % Interference with side plane surface
                if Struts.door.dd(i,k) ~= 0 
                    CD_s(i,j) = CD_strut_K3(CD_s(i,j),...
                        abs(Struts.door.dd(i,k)-Struts_y(i,j))-Struts.diameter(i,j)*0.5,...
                        Struts.diameter(i,j)*0.5*(1+Struts.aspect(i,j)),...
                    strut_R3);
                end
            end 
            % Interference between struts
            if j ~= 1
                for k = 1:j-1
                    if abs(Struts_x(i,k) - Struts_x(i,j))< 0.5*(Struts.diameter(i,k)*Struts.aspect(i,k)+Struts.diameter(i,j)*Struts.aspect(i,j))
                        x = 0;
                    else
                        x = Struts_x(i,k) - Struts_x(i,j);
                    end
                    if abs(Struts_y(i,k) - Struts_y(i,j))< 0.5*(Struts.diameter(i,k)+Struts.diameter(i,j))
                        y = 0;
                    else
                        y = abs(Struts_x(i,k) - Struts_x(i,j));
                    end
                    [CD_s(i,k),CD_s(i,j)] = CD_inter_strut(CD_s(i,k),...
                        Struts.diameter(i,k)*0.5*(1+Struts.aspect(i,k)),...
                        CD_s(i,j),...
                        Struts.diameter(i,j)*0.5*(1+Struts.aspect(i,j)),...
                        x,...
                        y,...
                        strut_R2);
                end
            end
            
        end
    end
    % Cd of doors 
   
    for k = 1:size(Struts.door.dd,2) % number of doors, ie number of columns
        if Struts.door.td(i,k) ~= 0
            CD_d(i,k) = CD_door(Struts.door.td(i,k),...
                Struts.dZsection(i),...
                Struts.door.ld(i,k));
            % Drag of undercarriage door
            % td:   tickness of the door
            % wd:   witdh of the door
            % ld:   length of the door

            % Length of the door
            L_door(k) = L_door(k) + Struts.dZsection(i);
        else
            CD_d(i,k) = 0;
        end   
    end
    % Section postion compared to wheel main axis 
    % This is an approximation. should depend on j as well (phi)
    if i == 1
        Zsection(i)= Wheel.dt/2 + cosd(Struts.phi(i,1))*Struts.dZsection(i)/2;
        
    else
        Zsection(i) = Zsection(i-1) + cosd(Struts.phi(i,1))*Struts.dZsection(i)/2;
    end
end

M_gear = M_gear + dot(Zsection,sum(CD_s,2)+sum(CD_d,2) );

% Cd of cavity
CD_c = CD_cavity(max(Struts.door.ld(:)),... 
    sum(L_door(:)),... 
    max(abs(Struts_y(:)))*1.1,...
    CD_cav);
% Drag of undercarriage bay cavity
% lc:   lenght measured along the flow direction 
% wc:   width of the door
% hc:   depth of the cavity

M_gear = M_gear + CD_c*(Zsection(end)+Struts.dZsection(i)/2);

if tw ~= 0
    % Installation drag factor due to wing thickness
    F_wing = F1_wing(tw,cw,sum(Struts.dZsection(i))+ Wheel.dt,F1_coeff);
    % tw:   wing thickness
    % cw:   wing chord
    % z:    overall depth of overall undercarriage 

    % Installation drag factor due to flap
    F_flap = F2_flap(delta_F,c_f2w,cw,sum(Struts.dZsection(i))+ Wheel.dt,F2_coeff);
    % Installation drag factor due to flap
    % delta_F:  flap deflection
    % c_f2w:    ratio of flap chord to wing chord measured at the spanwise
    % location
    % cw:       wing chord
    % z:        overall depth of overall undercarriage 
else
    F_wing = 1;
    F_flap = 1;
end

% Calculation of Aero centre ref to wheel
z_aero = M_gear/(CD_w + sum(CD_s(:)) + sum(CD_d(:)) + CD_c)/cosd(Struts.phi(1,1))+Wheel.dt/2;

% Total Cd
CD_total = (CD_w+sum(CD_s(:)) + sum(CD_d(:)) + CD_c) * F_wing * F_flap;
end

%% General functions for drag estimation of retractable undercarriage
% 

function CD = CD_wi(dt,b,Theta,m,n,Re_m,CD_wheel1,CD_wheel2) 
% Drag of single (or multi) wheel with strut effect
% dt:       Total deth of wheel combination normal to freestream direction
% Diameter of wheel
% b:        Width of wheel or maximum width of wheel combination
% Theta:    Angle between strut and plane normal to wheel axle
% m:        Depth of gap between inner wheels of multi-wheel conf
% n:        Width of gap between inner wheels of multi-wheel conf
% Re_m:     Reynolds number

% critical Reynolds number
Re_crit = 5.0e5;

if    Re_m*dt < Re_crit 
    Cd0 = 1.2;
    Cdi = interp1(CD_wheel1.x, CD_wheel1.y, dt/b);
else
    if abs(Theta) < 45
        Cd0 = 0.55+ 0.1*(cosd(2*Theta))^2;
    else
        Cd0 = 0.55;
    end
    
    Cdi = interp1(CD_wheel2.x, CD_wheel2.y, dt/b);
end 
CD = Cd0*Cdi*(dt*b-m*n);
end 

function CD = CD_wp(dw,Hw,dt,b,Theta,m,n,Re_m,CD_wheel1,CD_wheel2,K_plane_wheel) 
% Drag of single (or multi) wheel with strut effect and plane surface
% dw:       Diameter of a wheel
% Hw:       Distance between cylindrical compoment of circular cross 
%   section and plane surface 
% dt:       Total deth of wheel combination normal to freestream direction
%   Diameter of wheel
% b:        Width of wheel or maximum width of wheel combination
% Theta:    Angle between strut and plane normal to wheel axle
% m:        Depth of gap between inner wheels of multi-wheel conf
% n:        Width of gap between inner wheels of multi-wheel conf

p = interp1(K_plane_wheel.x,K_plane_wheel.y,Hw/dw);
CD = p*CD_wi(dt,b,Theta,m,n,Re_m,CD_wheel1,CD_wheel2);

end

function CD = CD_si_circ(ls,ds,phi)
% Drag calculation of an isolated circular strut at an angle phi with the 
% flow
% ls:       Strut length
% ds:       Strut diameter
% phi:      angle with the flow

CD = 1.2*ls*ds*(cosd(abs(phi)))^3;
end

function CD = CD_si_rect(lsr,csr,wsr,phi,CD_strut)
% Drag calculation of an isolated rectangular strut at an angle phi with 
% the flow
% lsr:      Strut length
% csr:      Strut chord normal to its axis
% wsr:      Strut width
% phi:      angle with the flow
CDsr = interp1(CD_strut.x, CD_strut.y,csr/wsr);
CD = CDsr*lsr*wsr*(cosd(abs(phi)))^3;
end

function [CD1,CD2] = CD_inter_strut(CD1i,d1,CD2i,d2,x,y,strut_R2)
% Interference calculation due to presence of several struts 
% CD1:      Drag of first strut 
% d1:       Diameter of first strut 
% CD2:      Drag of second 
% d2:       Diameter of second strut 
% x:        Mean position of second strut with regards to the first one in
%   the flow direction
% y:        Mean position of the second strut with regards to the first one 
%   normal to the flow direction

if x < 0
    if d1 > d2
        CD1io = CD1i;
        CD2io = CD2i*interp1(strut_R2.down.x, strut_R2.down.y,abs(-1+x/mean(d1+d2)));
    elseif d1 < d2
        CD1io = CD1i*interp1(strut_R2.up.x, strut_R2.up.y,abs(-1+x/mean(d1+d2)));
        CD2io = CD2i;
    else
        CD1io = CD1i*interp1(strut_R2.up.x, strut_R2.up.y,abs(-1+x/mean(d1+d2)));
        CD2io = CD2i*interp1(strut_R2.down.x, strut_R2.down.y,abs(-1+x/mean(d1+d2)));
    end
elseif x > 0 
    if d1 > d2    
        CD1io = CD1i;
        CD2io = CD2i*interp1(strut_R2.up.x, strut_R2.up.y,1+x/mean(d1+d2));
    elseif d1 < d2
        CD1io = CD1i*interp1(strut_R2.down.x, strut_R2.down.y,1+x/mean(d1+d2));
        CD2io = CD2i;
    else
        CD1io = CD1i*interp1(strut_R2.down.x, strut_R2.down.y,1+x/mean(d1+d2));
        CD2io = CD2i*interp1(strut_R2.up.x, strut_R2.up.y,1+x/mean(d1+d2));
    end
else
    CD1io = CD1i;
    CD2io = CD2i;    
end
    
if abs(y) > 0
    if d1 > d2
        CD1 = CD1io*interp1(strut_R2.side.xhigh, strut_R2.side.yhigh,...
            1+abs(y)/mean(d1+d2));
        CD2 = CD2io*interp1(strut_R2.side.xlow, strut_R2.side.ylow,...
            1+abs(y)/mean(d1+d2));
    elseif d1 < d2
        CD1 = CD1io*interp1(strut_R2.side.xlow, strut_R2.side.ylow,...
            1+abs(y)/mean(d1+d2));
        CD2 = CD2io*interp1(strut_R2.side.xhigh, strut_R2.side.yhigh,...
            1+abs(y)/mean(d1+d2));
    else
        K2 = interp1(strut_R2.side.xmean, strut_R2.side.ymean, ...
            1+abs(y)/mean(d1+d2));
        CD1 = CD1io* K2;
        CD2 = CD2io* K2;  
    end
        
else
    CD1 = CD1io;
    CD2 = CD2io;
end

end 

function CD = CD_strut_K3(CDi,Hs,ds,strut_R3)
% Strut drag interference factor, R3, due to the presence of plane surface
% CDi:      CD of strut before correction
% Hs:       Distance between plane surface and axis of strut
% ds:       Diameter of the strut

CD = CDi*interp1(strut_R3.x,strut_R3.y,abs(Hs)/ds,'linear','extrap');

end

function CD = CD_door(td,wd,ld)
% Drag of undercarriage door
% td:   tickness of the door
% wd:   witdh of the door
% ld:   length of the door

C_Dd = 0.5; % drag coeff
C_Fd = 0.0033; % mean skin frinction coeff

CD = (C_Dd * td * wd + C_Fd * ld * wd); % Neglect skin friction drag

end

function CD = CD_cavity(lc,wc,hc,CD_cavity)
% Drag of undercarriage bay cavity
% lc:   lenght measured along the flow direction
% wc:   width of the door
% hc:   depth of the cavity

if wc/hc <= 1.5 
    CD = interp1(CD_cavity.wch1.x,CD_cavity.wch1.y,hc/lc)*lc*wc;
elseif wc/hc <= 3.0 
    CD = interp1(CD_cavity.wch2.x,CD_cavity.wch2.y,hc/lc)*lc*wc;
elseif wc/hc <= 5.25 
    CD = interp1(CD_cavity.wch4.x,CD_cavity.wch4.y,hc/lc)*lc*wc;
else
    CD = interp1(CD_cavity.wch6.x,CD_cavity.wch6.y,hc/lc)*lc*wc;
end
end

function F1 = F1_wing(tw,cw,z,F1_coeff)
% Installation drag factor to wing thickness
% tw:   wing thickness
% cw:   wing chord
% z:    overall depth of overall undercarriage 

F1 = 1 + interp1(F1_coeff.x,F1_coeff.y,z/cw)*tw/cw;
end

function F2 = F2_flap(delta_F,c_f2w,cw,z,F2_coeff)
% Installation drag factor due to flap
% delta_F:  flap deflection
% c_f2w:    ratio of flap chord to wing chord measured at the spanwise
% location
% cw:       wing chord
% z:        overall depth of overall undercarriage 

F2 = ( 1 -(0.0186 - 0.018*(0.5*z/cw - 0.2) +0.0053*(0.5*z/cw - 0.2)^2)*...
    interp1(F2_coeff.x,F2_coeff.y,c_f2w)*delta_F)^2;
end

%% General functions drag estimation of a fixed undercarriage 
% 

function CD = CD_wfairing(Sf,b,dw,hw,CD_wheel1)
% Drag of wheel with fairings 
% Sf:       frontal area of the fairing
% dw:       diameter of wheel
% b:        width of wheel
% hw:       depth of the exposed wheel

CD = 0.007*Sf + 0.55*interp1(CD_wheel1.x ,CD_wheel1.y ,dw/b)*...
    b*hw;
end







