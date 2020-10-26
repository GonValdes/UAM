function [y_distribution,c_distribution,beta_distribution,thickness_distribution,camber_distribution]=qprop_analysisv3(prop,cond,dir,run,filename,AerofoilDataset)

%Function for the analysis of a defined propeller geometry using QPROP: 
%creates a txt with the geometry of the propeller

%Inputs
% prop: propeller geometry data (defined in Qprop file)
% cond: fligh condition data (defined in Qprop file)
% dir: directories, at least .qprop(qprop.exe directory) and .data(results)
%   must be defined 
% airfoil_data: data obtained from script xfoil_curves, contains
%   CL0,CLalfa...
% run: data for the different cases of rpm and v wanted to test (defined in
%    Qprop file)
%aerofoil_database: database with the different NACA airfoils inside
%Created by Gonzalo Valdes 01/Apr/2020

%% Create conditions file 
fid = fopen('qcon.def', 'w');
fprintf(fid,strcat(num2str(cond.rho),'    ! rho (kg/m^3)   density'));
fprintf(fid,'\n');
fprintf(fid,strcat(num2str(cond.mu),'    ! mu  (kg/m-s)   dynamic viscosity'));
fprintf(fid,'\n');
fprintf(fid,strcat(num2str(cond.a),'    ! a   (m/s)      speed of sound'));
fclose(fid);
%% QPROP file
filename.prop = 'propfile';

%Calculate radius point that will define pchip distribution
x = [prop.radius_root prop.radius1*prop.radius_tip...
    prop.radius2*prop.radius_tip prop.radius_tip];%radius points

% chord distribution cubic piecewise, pchip
y_chord = [prop.cr prop.chord_rad1*prop.cr...
    prop.chord_rad2*prop.cr prop.ct];%chord points

y_distribution = linspace(x(1),x(end),10);
c_distribution = pchip(x,y_chord,y_distribution);

% beta distribution cubic piecewise, pchip

y_beta = [1 prop.beta_rad1 prop.beta_rad2 prop.beta_tip]*prop.beta_root;
beta_distribution = pchip(x,y_beta,y_distribution);


% camber distribution cubic piecewise, pchip

y_camber =[1 prop.afcamber_rad1 prop.afcamber_rad2 prop.afcamber_tip]...
        *prop.afcamber_root;
camber_distribution = spline(x,y_camber,y_distribution);

% thickness distribution cubic piecewise, pchip

y_thickness = [1 prop.afthickness_rad1 prop.afthickness_rad2 prop.afthickness_tip]...
    *prop.afthickness_root;
thickness_distribution = spline(x,y_thickness,y_distribution);

[airfoil_data] = GetAerofoilData(camber_distribution(1),thickness_distribution(1),prop.MaxCamberPos,AerofoilDataset);

    
%% Write Geometry file
fid = fopen(strcat(filename.prop,'.txt'), 'w');
fprintf(fid,'Propeller \n \n');
fprintf(fid,strcat(num2str(prop.n_blades),'    !Number of blades'));
fprintf(fid,'\n');
fprintf(fid,strcat(num2str(airfoil_data.CL0),32,32,num2str(airfoil_data.CLa*180/pi),'    !CL0  CL_a(rad-1)'));
fprintf(fid,'\n');
fprintf(fid,strcat(num2str(airfoil_data.CLmin),32,32,num2str(airfoil_data.CLmax),'    !CLmin  CL_max'));
fprintf(fid,'\n \n');
fprintf(fid,strcat(num2str(airfoil_data.CD0),32,32,num2str(airfoil_data.CD2u),32,32,num2str(airfoil_data.CD2l),32,32,num2str(airfoil_data.CLCD0),'    !CD0 CD2u CD2l CLCD0'));
fprintf(fid,'\n');
fprintf(fid,strcat(num2str(10^6),32,32,num2str(-0.2),'    !Re_ref  prop.Re_exp'));
fprintf(fid,'\n \n');
fprintf(fid,strcat('1.0',32,32,'1.0',32,32,'1.0','    !Rfac Cfac Bfac'));%scaling factors in case of different units
fprintf(fid,'\n');
fprintf(fid,strcat('0.',32,32,'0.',32,32,'0.','    !Radd Cadd Badd'));%bias
fprintf(fid,'\n \n');

%geometry definition
fprintf(fid,'# r(m)   chord(m)   beta(deg)  CL0  CLa  CLmin  CLmax  CD0  CD2u  CD2l  CLCD0');
fprintf(fid,'\n');


for i = 1:length(y_distribution)
    
   %Calculate aerofoil at each section
%     aerofoil = strcat(num2str(camber_distribution(i)) num2str(prop.MaxCamberPos(i)) ...
%                     num2str(thickness_distribution(i),'%02.f'))

    [aerofoil] = GetAerofoilData(camber_distribution(i),thickness_distribution(i),prop.MaxCamberPos,AerofoilDataset);

    fprintf(fid,[num2str(y_distribution(i)),32,32,num2str(c_distribution(i))...
        ,32,32,num2str(beta_distribution(i)),32,32,num2str(aerofoil.CL0),32,32 ...
        ,num2str(aerofoil.CLa*180/pi),32,32,num2str(aerofoil.CLmin),32,32 ...
        ,num2str(aerofoil.CLmax),32,32,num2str(aerofoil.CD0),32,32 ...
        ,num2str(aerofoil.CD2u),32,32,num2str(aerofoil.CD2l),32,32 ...
        ,num2str(aerofoil.CLCD0), '\n']);
end
fclose(fid);


%% Execute qprop
dos(['cd ' dir.qprop ' | qprop.exe propfile.txt neumotor.txt ' strcat(num2str(run.V),' 0 0 0 ',32,num2str(run.T)) ' > ' strcat(filename.qprop_results,'.txt')]);

%Move files to qprop folder
% delete(strcat(filename.prop,'.txt'));
% movefile('propfile.txt',dir.data);
end

function [aerofoil] = GetAerofoilData(camber,thickness,MaxCamberPos,AerofoilDataset)
aerofoil.CL0 = interpn(AerofoilDataset.camber_range,AerofoilDataset.thickness_range...
    ,AerofoilDataset.MaxCamberPos_range...% XYZ breakpoints
    ,AerofoilDataset.CL0...%Value at the breakpoint
    ,camber,thickness,MaxCamberPos);
aerofoil.CLa = interpn(AerofoilDataset.camber_range,AerofoilDataset.thickness_range...
    ,AerofoilDataset.MaxCamberPos_range...% XYZ breakpoints
    ,AerofoilDataset.CLa...%Value at the breakpoint
    ,camber,thickness,MaxCamberPos);
aerofoil.CLmax = interpn(AerofoilDataset.camber_range,AerofoilDataset.thickness_range...
    ,AerofoilDataset.MaxCamberPos_range...% XYZ breakpoints
    ,AerofoilDataset.CLmax...%Value at the breakpoint
    ,camber,thickness,MaxCamberPos);
aerofoil.CLmin = interpn(AerofoilDataset.camber_range,AerofoilDataset.thickness_range...
    ,AerofoilDataset.MaxCamberPos_range...% XYZ breakpoints
    ,AerofoilDataset.CLmin...%Value at the breakpoint
    ,camber,thickness,MaxCamberPos);
aerofoil.CD0 = interpn(AerofoilDataset.camber_range,AerofoilDataset.thickness_range...
    ,AerofoilDataset.MaxCamberPos_range...% XYZ breakpoints
    ,AerofoilDataset.CD0...%Value at the breakpoint
    ,camber,thickness,MaxCamberPos);
aerofoil.CLCD0 = interpn(AerofoilDataset.camber_range,AerofoilDataset.thickness_range...
    ,AerofoilDataset.MaxCamberPos_range...% XYZ breakpoints
    ,AerofoilDataset.CLCD0...%Value at the breakpoint
    ,camber,thickness,MaxCamberPos);
aerofoil.CD2l = interpn(AerofoilDataset.camber_range,AerofoilDataset.thickness_range...
    ,AerofoilDataset.MaxCamberPos_range...% XYZ breakpoints
    ,AerofoilDataset.CD2l...%Value at the breakpoint
    ,camber,thickness,MaxCamberPos);
aerofoil.CD2u = interpn(AerofoilDataset.camber_range,AerofoilDataset.thickness_range...
    ,AerofoilDataset.MaxCamberPos_range...% XYZ breakpoints
    ,AerofoilDataset.CD2u...%Value at the breakpoint
    ,camber,thickness,MaxCamberPos);
end
