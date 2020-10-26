function generate_AVLcase(filename,flight_con,inertia,mass,CG)
%%  generate_AVLcase(casename,flightcon,wing)
%   Code to generate AVL run file based on the following
%   information obtained by running 'case_definition.m':
%
%   inertia: structure with Ixx,Iyy,Izz and Ixz components of inertia matrix
%   wing - wing data [structure]
%   flightcon - defined flight condition [structure]
%
%   The function generates the AVL geometry file : casename.run
%
%   Created by: Mudassir Lone
%   Created Date: 2017-05-20
%   Last modified by: Gonzalo Valdes
%   Last modified Date: 2020-03-28
%   Description of modification: Code adapted to Urban air mobility project
%% Write file
string{1} = '---------------------------------------------';
string{2} = ['Run case  1: ' filename '\n'];

string{3} = ['alpha        ->  alpha     =   0 deg'];
string{4} = ['beta         ->  beta        =   0.0' ];
string{5} = ['pb/2V        ->  pb/2V       =   0.0' ];
string{6} = ['qc/2V        ->  qc/2V       =   0.0' ];
string{7} = ['rb/2V        ->  rb/2V       =   0.0' ];
string{8} = ['' ];    
string{9} = [' ' ];    
string{10} = [' ' ];    
string{11} = ['' ]; 
string{12} = ['alpha      =   0 deg'];
string{13} = ['beta      =   0 deg'];
string{14} = ['pb/2V     =   0.00000     '];
string{15} = ['qc/2V     =   0.00000     '];
string{16} = ['rb/2V     =   0.00000     '];
string{17} = [' '];
string{18} = [''];
string{19} = ['bank      =   0.00000     deg'];
string{20} = ['elevation =   0.00000     deg'];
string{21} = ['heading   =   0.00000     deg'];
string{22} = ['Mach      =   ' num2str(flight_con.Mach)];
string{23} = ['velocity  =   ' num2str(flight_con.TAS) '  m/s'];
string{24} = ['density   =   ' num2str(flight_con.rho) '  kg/m^3'];
string{25} = ['grav.acc. =   9.81   m/s^2'];
string{26} = ['turn_rad. =   0.00000     m'];
string{27} = ['load_fac. =   1.00000     '];
string{28} = ['X_cg      =   ' num2str(CG) ' m'];
string{29} = ['Y_cg      =   0.00 m'];
string{30} = ['Z_cg      =   0.00 m'];
string{31} = ['mass      =   ' num2str(mass) ' kg'];
string{32} = ['Ixx       =   ' num2str(inertia.Ixx) ' kg-m^2'];
string{33} = ['Iyy       =   ' num2str(inertia.Iyy) ' kg-m^2'];
string{34} = ['Izz       =   ' num2str(inertia.Izz) ' kg-m^2'];
string{35} = ['Ixy       =   0.0000 kg-m^2'];
string{36} = ['Iyz       =   0.0000 kg-m^2'];
string{37} = ['Izx       =   ' num2str(inertia.Ixz) ' kg-m^2'];
% string{38} = ['visc CL_a =   0.00000 '];
% string{39} = ['visc CL_u =   0.00000 '];
% string{40} = ['visc CM_a =   0.00000 '];
% string{41} = ['visc CM_u =   0.00000 '];

% Write to geometry file
fid = fopen([filename '.run'],'w');
for i=1:size(string,2)
    fprintf(fid,string{i});
    fprintf(fid,'\n');
end

fclose(fid);


return