function  [CD0,CD] = getCD0(geo,Re,Mach)
%Prediction of zero-lift drag coefficient. From Rayner(1998),
%Torenbeek(1998),DATCOM (1978)
 
%Thickness for each component 0.xx
thickr_fus = str2num(geo.airfoil_fus(3:4))/100;
thickr_w = str2double(geo.airfoilroot_w(3:4))/100;
thickratio_w = str2double(geo.airfoiltip_w(3:4))/str2double(geo.airfoilroot_w(3:4));
thickmean_w = (str2double(geo.airfoiltip_w(3:4))+str2double(geo.airfoilroot_w(3:4)))/(2*100);
thickr_c = str2double(geo.airfoilroot_c(3:4))/100;
thickratio_c = 1;
thickr_fin = str2double(geo.airfoil_f(3:4))/100;
thickratio_fin = 1;
%maximum thickness position
maxthick_fus = str2num(geo.airfoil_fus(2))/10;
maxthick_fin = str2num(geo.airfoil_f(2))/10;
maxthick_w = str2num(geo.airfoilroot_w(2))/10;
maxthick_c = str2num(geo.airfoilroot_c(2))/10;

%Wetted area from Torenbeek
S_wet_fus = 2*geo.S_fus*(1+0.25*thickr_fus*(1+1)/(1+1));
S_wet_w =2*geo.S_w*(1+0.25*thickr_w*(1+geo.taper_w*thickratio_w)/(1+geo.taper_w));
S_wet_c =2*geo.S_c*(1+0.25*thickr_c*(1+geo.taper_c*thickratio_c)/(1+geo.taper_c));
S_wet_fin =2*geo.S_f*(1+0.25*thickr_fin*(1+geo.taper_f*thickratio_fin)/(1+geo.taper_f));

%Friccion coefficient from Blasius(Laminar) and DATCOM(turbulent)
perc_laminar=0.2;%define percentage of laminar flow
Cf_w = Friction_coefficient(Re,geo.mac_w,geo.mac_w,perc_laminar,Mach);
Cf_fus = Friction_coefficient(Re,geo.mac_w,geo.Cr_fus,perc_laminar,Mach);
Cf_c = Friction_coefficient(Re,geo.mac_w,geo.mac_c,perc_laminar,Mach);
Cf_fin = Friction_coefficient(Re,geo.mac_w,geo.mac_f,perc_laminar,Mach);

%Form Factor from DATCOM
FF_w = (1+ 0.6/maxthick_w*thickmean_w + 100*thickmean_w^4)*(1.34*Mach^0.18*(cosd(geo.sweep1_w))^0.28);
FF_c = (1+ 0.6/maxthick_c*thickr_c + 100*thickr_c^4)*(1.34*Mach^0.18*(cosd(geo.sweep1_c))^0.28);
FF_fus = (1+ 0.6/maxthick_fus*thickr_fus + 100*thickr_fus^4)*(1.34*Mach^0.18);
FF_fin = (1+ 0.6/1*thickr_fin + 100*thickr_fin^4)*(1.34*Mach^0.18*(cosd(geo.sweep1_f))^0.28);

%Final CD0 calculation CD0 = sum(Cf_i*FF_i*Swet_i/Sref)
CD0 = Cf_w*FF_w*S_wet_w/geo.S_w + Cf_fus*FF_fus*S_wet_fus/geo.S_w +...
    Cf_c*FF_c*S_wet_c/geo.S_w + Cf_fin*FF_fin*S_wet_fin/geo.S_w;

CD.w = Cf_w*FF_w*S_wet_w/geo.S_w;
CD.c = Cf_c*FF_c*S_wet_c/geo.S_w;
CD.fus = Cf_fus*FF_fus*S_wet_fus/geo.S_w;
end

function Cf = Friction_coefficient(Re_ref,l_ref,l_new,perc_laminar,Mach)
%Friction coefficient depends of Reynolds, but each component has each own
%Reynolds. Script to account for that.
Re = Re_ref / l_ref * l_new;
Cf_laminar = 1.328/(Re^0.5);
Cf_turb = 0.455/(log10(Re)^2.58 * (1+0.144*Mach^2)^0.65);

Cf = perc_laminar*Cf_laminar + (1-perc_laminar) * Cf_turb;
end