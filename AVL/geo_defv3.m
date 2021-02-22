function geo_defv3(filename,geo,initialise)
%   Code to generate AVL run file based on the following
%   information obtained by running 'UAM_AVL.m':
%
%   filename - At least includind filename.geo. String with the name of the avl geo file
%   geo - structure with all the aircraft parameters necessary to define geometry

%   The function generates the AVL geometry file : filename.avl
%
%   Created by: Gonzalo Valdes
%   Created Date: 2020-Apr
%   Reference : http://web.mit.edu/drela/Public/web/avl/avl_doc.txt

fid = fopen(strcat(filename,'.avl'), 'w');

fprintf(fid,['Urban Air Mobility \n']);
fprintf(fid,['#Mach \n']);
fprintf(fid,[' 0 \n']);
fprintf(fid,['#IYsym    IZsym   Zsym \n']);
fprintf(fid,[' 0        0       0 \n']);
fprintf(fid,['#Sref    Cref    Bref   \n']);
fprintf(fid,[num2str(geo.S_w) '    ' num2str(geo.mac_w) '    ' num2str((geo.b_w+geo.b_fus)*2) ' \n']);
fprintf(fid,['#Xref    Yref    Zref \n']);
fprintf(fid,[num2str(initialise.CG) '     0       0 \n ']);


fprintf(fid,['#======================================= \n']);
fprintf(fid,['SURFACE \n']);%Here we define the WING
fprintf(fid,['WING \n']); 
fprintf(fid,['#Nchordwise Cspace [Nspanwise Sspace] \n']);
fprintf(fid,['24            1.0       30         1.0\n ']);
fprintf(fid,['YDUPLICATE \n']);
fprintf(fid,['0 \n']);
fprintf(fid,['TRANSLATE \n']);
fprintf(fid,[num2str(geo.xle_w) '  ' num2str(geo.yle_w) '  ' num2str(geo.zle_w) ' \n']);
fprintf(fid,['#--------------------------------------- \n']);
fprintf(fid,['Section \n']);%root of the wing
fprintf(fid,['#Xle    Yle    Zle     Chord   Ainc \n']);
fprintf(fid,['0  0  0  ' num2str(geo.Cr_w) '  ' num2str(geo.aoar_w) ' \n']);
fprintf(fid,['NACA \n']);
fprintf(fid,[geo.airfoilroot_w ' \n']);
fprintf(fid,['CLAF \n']);
fprintf(fid,[num2str(1+0.77/100*value_at_yposition(0,str2num(geo.airfoilroot_w(3:4)),str2num(geo.airfoiltip_w(3:4)),geo.b_w)) ' \n']);
fprintf(fid,['#--------------------------------------- \n']);
fprintf(fid,['Section \n']);%start of the outer aileron of the wing
fprintf(fid,['#Xle    Yle    Zle     Chord   Ainc \n']);
fprintf(fid,[num2str(geo.b_w*geo.innery_outerail/100*tan(geo.sweep1_w*pi/180)) '  '...
    num2str(geo.b_w*geo.innery_outerail/100) '  0  '  num2str(value_at_yposition(geo.innery_outerail,geo.Cr_w,geo.Ct_w,geo.b_w))...
    '  ' num2str(value_at_yposition(geo.innery_outerail,geo.aoar_w,geo.aoat_w,geo.b_w)) '\n']);
fprintf(fid,['NACA \n']);
fprintf(fid,[geo.airfoiltip_w ' \n']);
fprintf(fid,['CLAF \n']);
fprintf(fid,[num2str(1+0.77/100*value_at_yposition(geo.innery_outerail,str2num(geo.airfoilroot_w(3:4)),str2num(geo.airfoiltip_w(3:4)),geo.b_w)) ' \n']);;
fprintf(fid,['#Cname   Cgain  Xhinge  HingeVec     SgnDup \n']);%Aileron
fprintf(fid,['CONTROL \n']);
fprintf(fid,['outer_ail  1.0  ' num2str(geo.x_outerail/100) '   0. 1. 0.   -1  \n']);%Aileron
fprintf(fid,['#--------------------------------------- \n']);
fprintf(fid,['Section \n']);%end of the outer aileron of the wing
fprintf(fid,['#Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace \n']);
fprintf(fid,[num2str(geo.b_w*geo.outery_outerail/100*tan(geo.sweep1_w*pi/180)) '  '...
    num2str(geo.b_w*geo.outery_outerail/100) '  0  '  num2str(value_at_yposition(geo.outery_outerail,geo.Cr_w,geo.Ct_w,geo.b_w))...
    '  ' num2str(value_at_yposition(geo.outery_outerail,geo.aoar_w,geo.aoat_w,geo.b_w)) '\n']);
fprintf(fid,['NACA \n']);
fprintf(fid,[geo.airfoiltip_w ' \n']);
fprintf(fid,['CLAF \n']);
fprintf(fid,[num2str(1+0.77/100*value_at_yposition(geo.outery_outerail,str2num(geo.airfoilroot_w(3:4)),str2num(geo.airfoiltip_w(3:4)),geo.b_w)) ' \n']);
fprintf(fid,['#Cname   Cgain  Xhinge  HingeVec     SgnDup \n']);%Aileron
fprintf(fid,['CONTROL \n']);
fprintf(fid,['outer_ail  1.0  ' num2str(geo.x_outerail/100) '   0. 1. 0.   -1 \n']);%Aileron
fprintf(fid,['#--------------------------------------- \n']);
fprintf(fid,['Section \n']);%tip of the wing
fprintf(fid,['#Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace \n']);
fprintf(fid,[num2str(geo.b_w*tan(geo.sweep1_w*pi/180)) '  ' num2str(geo.b_w) '  0  '  num2str(geo.Ct_w) '  ' num2str(geo.aoat_w) '\n']);
fprintf(fid,['NACA \n']);
fprintf(fid,[geo.airfoiltip_w ' \n']);
fprintf(fid,['CLAF \n']);
fprintf(fid,[num2str(1+0.77*value_at_yposition(100,str2num(geo.airfoilroot_w(3:4)),str2num(geo.airfoiltip_w(3:4)),geo.b_w)/100) ' \n']);

fprintf(fid,['#======================================= \n']);
fprintf(fid,['Surface \n']);%Here we define the CANARD
fprintf(fid,['Canard \n']); 
fprintf(fid,['#Nchordwise Cspace [Nspanwise Sspace] \n']);
fprintf(fid,['15           1.0       20         1.0\n ']);
fprintf(fid,['YDUPLICATE \n']);
fprintf(fid,['0 \n']);
fprintf(fid,['TRANSLATE \n']);
fprintf(fid,[num2str(geo.xle_c) '  ' num2str(geo.yle_c) '  ' num2str(geo.zle_c) ' \n']);
fprintf(fid,['#--------------------------------------- \n']);
fprintf(fid,['Section \n']);%root of the canard
fprintf(fid,['#Xle    Yle    Zle     Chord   Ainc \n']);
fprintf(fid,['0  0  0  ' num2str(geo.Cr_c) '  ' num2str(geo.aoar_c) ' \n']);
fprintf(fid,['NACA \n']);
fprintf(fid,[geo.airfoilroot_c ' \n']);
fprintf(fid,['CLAF \n']);
fprintf(fid,[num2str(1+0.77*str2num(geo.airfoilroot_c(3:4))/100) ' \n']);
fprintf(fid,['#--------------------------------------- \n']);
fprintf(fid,['Section \n']);%start of the inner aileron of the canard, which is an elevator
fprintf(fid,['#Xle    Yle    Zle     Chord   Ainc \n']);
fprintf(fid,['0  ' num2str(geo.b_c*geo.innery_innerail/100) '  0  '...
      num2str(geo.Cr_c) '  '  num2str(geo.aoar_c)  ' \n']);
fprintf(fid,['NACA \n']);
fprintf(fid,[geo.airfoilroot_c ' \n']);
fprintf(fid,['CLAF \n']);
fprintf(fid,[num2str(1+0.77*str2num(geo.airfoilroot_c(3:4))/100) ' \n']);
fprintf(fid,['#Cname   Cgain  Xhinge  HingeVec     SgnDup \n']);%Elevator
fprintf(fid,['CONTROL \n']);
fprintf(fid,['inner_ail  1.0  ' num2str(geo.x_innerail/100) '   0. 1. 0.   1  \n']);%Elevator
fprintf(fid,['#--------------------------------------- \n']);
fprintf(fid,['Section \n']);%end of the inner aileron of the canard, which is an elevator
fprintf(fid,['#Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace \n']);
fprintf(fid,['0  ' num2str(geo.b_c*geo.outery_innerail/100) '  0  '...
      num2str(geo.Cr_c) '  '  num2str(geo.aoar_c)  ' \n']);
fprintf(fid,['NACA \n']);
fprintf(fid,[geo.airfoilroot_c ' \n']);
fprintf(fid,['CLAF \n']);
fprintf(fid,[num2str(1+0.77*str2num(geo.airfoilroot_c(3:4))/100) ' \n']);
fprintf(fid,['#Cname   Cgain  Xhinge  HingeVec     SgnDup \n']);%Elevator
fprintf(fid,['CONTROL \n']);
fprintf(fid,['inner_ail  1.0  ' num2str(geo.x_innerail/100) '   0. 1. 0.   1 \n']);%Elevator
fprintf(fid,['#--------------------------------------- \n']);
fprintf(fid,['Section \n']);%tip of the canard
fprintf(fid,['#Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace \n']);
fprintf(fid,[num2str(geo.b_c*tan(geo.sweep1_c*pi/180)) '  ' num2str(geo.b_c) '  0  '  num2str(geo.Ct_c) '  ' num2str(geo.aoar_c) '\n']);
fprintf(fid,['NACA \n']);
fprintf(fid,[geo.airfoiltip_c ' \n']);
fprintf(fid,['CLAF \n']);
fprintf(fid,[num2str(1+0.77*str2num(geo.airfoiltip_c(3:4))/100) ' \n']);


fprintf(fid,['#======================================= \n']);
fprintf(fid,['SURFACE \n']);%Here we define the Right FIN
fprintf(fid,['FIN_R \n']); 
fprintf(fid,['#Nchordwise Cspace [Nspanwise Sspace] \n']);
fprintf(fid,['5            1.0       5         1.0\n ']);
fprintf(fid,['TRANSLATE \n']);
fprintf(fid,[num2str(geo.xle_f) '  ' num2str(geo.yle_f) '  ' num2str(geo.zle_f) ' \n']);
fprintf(fid,['#--------------------------------------- \n']);
fprintf(fid,['Section \n']);%root of the fin
fprintf(fid,['#Xle    Yle    Zle     Chord   Ainc \n']);
fprintf(fid,['0  0  0  ' num2str(geo.Cr_f) '  0 \n']);
fprintf(fid,['NACA \n']);
fprintf(fid,[geo.airfoil_f ' \n']);
fprintf(fid,['#Cname   Cgain  Xhinge  HingeVec     SgnDup \n']);
fprintf(fid,['CONTROL \n']);
fprintf(fid,['rudder_r  1  0   0. 0. 1.   0 \n']);%rudder
fprintf(fid,['#--------------------------------------- \n']);
fprintf(fid,['Section \n']);%tip of the fin
fprintf(fid,['#Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace \n']);
fprintf(fid,[num2str(geo.b_f*tan(geo.sweep1_f*pi/180)) '  0  '  num2str(geo.b_f) '  '  num2str(geo.Ct_f) '  0 \n']);
fprintf(fid,['NACA \n']);
fprintf(fid,[geo.airfoil_f ' \n']);
fprintf(fid,['#Cname   Cgain  Xhinge  HingeVec     SgnDup \n']);
fprintf(fid,['CONTROL \n']);
fprintf(fid,['rudder_r  1  0   0. 0. 1.   0 \n']);%rudder


fprintf(fid,['#======================================= \n']);
fprintf(fid,['SURFACE \n']);%Here we define the Left FIN
fprintf(fid,['FIN_L \n']); 
fprintf(fid,['#Nchordwise Cspace [Nspanwise Sspace] \n']);
fprintf(fid,['5            1.0       5         1.0\n ']);
fprintf(fid,['TRANSLATE \n']);
fprintf(fid,[num2str(geo.xle_f) '  ' num2str(-geo.yle_f) '  ' num2str(geo.zle_f) ' \n']);
fprintf(fid,['#--------------------------------------- \n']);
fprintf(fid,['Section \n']);%root of the fin
fprintf(fid,['#Xle    Yle    Zle     Chord   Ainc \n']);
fprintf(fid,['0  0  0  ' num2str(geo.Cr_f) '  0 \n']);
fprintf(fid,['NACA \n']);
fprintf(fid,[geo.airfoil_f ' \n']);
fprintf(fid,['#Cname   Cgain  Xhinge  HingeVec     SgnDup \n']);
fprintf(fid,['CONTROL \n']);
fprintf(fid,['rudder_l  1  0   0. 0. 1.   0 \n']);%rudder
fprintf(fid,['#--------------------------------------- \n']);
fprintf(fid,['Section \n']);%tip of the fin
fprintf(fid,['#Xle    Yle    Zle     Chord   Ainc  Nspanwise  Sspace \n']);
fprintf(fid,[num2str(geo.b_f*tan(geo.sweep1_f*pi/180)) '  0  '  num2str(geo.b_f) '  '  num2str(geo.Ct_f) '  0 \n']);
fprintf(fid,['NACA \n']);
fprintf(fid,[geo.airfoil_f ' \n']);
fprintf(fid,['#Cname   Cgain  Xhinge  HingeVec     SgnDup \n']);
fprintf(fid,['CONTROL \n']);
fprintf(fid,['rudder_l  1  0   0. 0. 1.   0 \n']);%rudder


fprintf(fid,['#======================================= \n']);
fprintf(fid,['SURFACE \n']);%Here we define the FUSELAGE
fprintf(fid,['FUSELAGE \n']); 
fprintf(fid,['#Nchordwise Cspace [Nspanwise Sspace] \n']);
fprintf(fid,['26            1.0       20         1.0\n ']);
fprintf(fid,['YDUPLICATE \n']);
fprintf(fid,['0 \n']);
fprintf(fid,['TRANSLATE \n']);
fprintf(fid,['0  0  0  \n']);
fprintf(fid,['#--------------------------------------- \n']);
fprintf(fid,['Section \n']);%root of the Fuselage
fprintf(fid,['#Xle    Yle    Zle     Chord   Ainc \n']);
fprintf(fid,['0  0  0  ' num2str(geo.Cr_fus) '  0 \n']);
fprintf(fid,['NACA \n']);
fprintf(fid,[geo.airfoil_fus ' \n']);
fprintf(fid,['CLAF \n']);
fprintf(fid,[num2str(1+0.77*str2num(geo.airfoil_fus(3:4))/100) ' \n']);
fprintf(fid,['#--------------------------------------- \n']);
fprintf(fid,['Section \n']);%tip of the fuselage
fprintf(fid,['#Xle    Yle    Zle     Chord   Ainc \n']);
fprintf(fid,['0  '...
    num2str(geo.b_fus) '  0  '  num2str(geo.Cr_fus) '  0 \n']);
fprintf(fid,['NACA \n']);
fprintf(fid,[geo.airfoil_fus ' \n']);
fprintf(fid,['CLAF \n']);
fprintf(fid,[num2str(1+0.77*str2num(geo.airfoil_fus(3:4))/100) ' \n']);

fclose(fid);
end

function [c] = value_at_yposition(pos,cr,ct,b)
%returns value of the component assuming linear behaviour
%pos in percentage %
%cr,ct,b in m
%function can also be used for angle of attack instead of chord
%change ct and cr for aoat and aoar
c = (ct-cr)*pos/100+cr;
end
