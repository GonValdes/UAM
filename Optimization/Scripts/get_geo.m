function [geo,initialise] = get_geo(pars)
%Using the geometry inputs, define the airframe geometry and obtain the
%estimation for the Oswald factor

%outer %MANOUEVRE
geo.innery_outerail = 60;%(%)
geo.outery_outerail = 90;%(%)
% Airframe geometry
%OPTIMIZED
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
%Manouevre longitudinal
geo.zle_c = pars(9);%(m)MANOUEVRE6
initialise.CG = pars(10);
%Manouevre lateral
geo.b_f= 0.8;%(m)

% FIXED parameters
geo.b_w= 4.1;
geo.twist_w = 0;%(deg);
geo.yle_w = 0.9;
geo.zle_w = 0;
%aileron parameters
%Canard elevator%MANOUEVRE
geo.innery_innerail = 10;%(%)
geo.outery_innerail = 90;%(%)
geo.x_innerail = 60;%(%)
geo.x_outerail = 75;%(%)
%canard
geo.Ct_c = geo.Cr_c;%(m)
geo.sweep1_c = 0;%(deg)
geo.twist_c = 0;%(deg)
geo.xle_c = 0;%(m)
geo.yle_c = 0.9;
geo.airfoilroot_c= '3412';
geo.airfoiltip_c = '3412';
%fin %MANOUEVRE
geo.sweep1_f = 15;%(deg)
geo.Cr_f = 0.5;%(m)
geo.xle_f = 9-geo.Cr_f;%(m)
geo.yle_f = 0.8;
geo.zle_f = 0.3;
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
geo.Cr_fus = 7;
geo.b_fus = 0.9;%the semi span
% geo.sweep_fus = 16.7;%deg
% geo.Ct_fus = geo.Cr_fus - geo.b_fus/tan(geo.sweep_fus*pi/180);
geo.airfoil_fus = '4421';%0421MOD this was cool also
geo.S_fus = geo.Cr_fus*geo.b_fus*2;

geo.aspect_ratio_w = ((geo.b_w+geo.b_fus)*2)^2/(geo.S_w);%+2*geo.b_fus*geo.Cr_w

%% Oswald factor definition. From Estimating the Oswald Factor... paper
geo.sweep25_w = abs(atan((geo.b_w*tan(geo.sweep1_w*pi/180)+geo.Ct_w/4-geo.Cr_w/4)/geo.b_w)*180/pi);
f1 = geo.taper_w - (-0.357 + 0.45*exp(0.0375*geo.sweep1_w));%(38)
f2 = 0.0524*f1^4 - 0.15*f1^3 + 0.1659*f1^2 - 0.0706*f1 + 0.0119;
e_w = 1/(1 + f2*geo.aspect_ratio_w);
e_fus = 1-2*(geo.b_fus/(geo.b_w+geo.b_fus))^2;
e_zld = 0.804;%zero-lift drag correction

geo.eOswald =e_w*e_fus*e_zld;
geo.K = 1/(pi*geo.aspect_ratio_w*geo.eOswald);
end