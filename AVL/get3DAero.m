function [ST_Data] = get3DAero(geofile,runfilename)
%%  [FR_Data,SB_Data,ST_Data] = get3DAeroV2(geofile,runfile)
%   This function runs the AVL case and returns:
%   FR_Data: Contents of the .fr file.
%   SB_Data: Contents of the .sb file.
%   ST_Data: Contents of the .st file.
%
%   Required inputs:
%   geofile - filename for the AVL geometry file
%   runfile - filename for the AVL run file
%
%   Outputs:
%   FR_Data - data for all surfaces and strips stored as a map
%   SB_Data - overall aerodynamic totals
%   ST_Data - stability derivatives
%
%   Created by: Mudassir Lone
%   Created Date: 2017-05-20
%   Reviewed By: Gonzalo Valdés
%   Reviewed Date: May-2020
%   Code adapted to UAM project 
%% Display simulation
%% INPUT variables
filename.geo = geofile;                 %'.\wing_AR6';
filename.run = [runfilename '.run'];                 %'.\wing_AR6_CASE1';
%Define a base file name to save
basename = geofile;
%% Create run file
%Open the file with write permission
fid = fopen(strcat(basename,'.avls'), 'w');

%Load the AVL definition of the aircraft
fprintf(fid, 'LOAD %s\n', strcat(filename.geo,'.avl'));
fprintf(fid, 'CASE %s\n', strcat(filename.run));
fprintf(fid, 'PLOP\ng\n\n');            %Disable Graphics
fprintf(fid, '%s\n',   'OPER');         %Open the OPER menu
fprintf(fid, 'O\n');
fprintf(fid, 'P\n');
fprintf(fid, 'T,T,T,F\n\n');
fprintf(fid, 'x\n');                    %Run the Case
% fprintf(fid, 'w\n');                    %Save the force-moment data
% fprintf(fid, strcat(basename, '.forces',' \n'));
fprintf(fid, 'st\n');           %Save the Stability derivaties data
fprintf(fid, strcat(basename, '.st',' \n'));
% fprintf(fid, 'sb\n');           %Save the Body-axis derivaties data
% fprintf(fid, strcat(basename, '.sb'));
fprintf(fid, '%s\n',   '');             %Drop out of OPER menu
% fprintf(fid, '\n'); 
% fprintf(fid, '\n'); 
% fprintf(fid, 'MODE\n');
% fprintf(fid, 'N\n');
% fprintf(fid, 'S\n');
% fprintf(fid, [runfilename '.sys'])
fprintf(fid, '\n');
fprintf(fid, '\n');                     %Exit MODE Menu
fprintf(fid, '\n');
fprintf(fid, 'Quit\n');                 %Quit Program
fclose(fid);                            %Close File

%% Execute Run
%Run AVL using
[~,result] = dos(strcat('.\avl.exe < ',basename,'.avls'));
% Read Output Data
% [SB_Data]  = Readfile_SB(basename);

[ST_Data]  = Readfile_ST(basename);

% [sys] = Readfile_Sys([runfilename '.sys']);

end
