%% Trimming for airspeed, altitude and vertical speed
% Obtain the trimming conditions for different flight conditions
% In this file trimming for variable tilt angle is included. Please contact
% Gonzalo if you want other conditions.

% First, load optimized aircraft data.

%% Establish the directories to work
% dir.work = 'D:\Cranfield\IRP\Coding\Qprop\Optimization';%folder to work in
dir.work = 'D:\Cranfield\IRP\Coding\Optimization';
dir.scripts_aux = 'D:\Cranfield\IRP\Coding\Optimization\Scripts';
dir.qprop = 'C:\Users\gonza\OneDrive\Escritorio\Full'; %qprop.exe folder
dir.AVL = dir.qprop;

addpath(dir.scripts_aux)
addpath(dir.work)
addpath('D:\Cranfield\IRP\Coding\Optimization\Results')
addpath('D:\misc_functs_matlab\ExportFigs')

filename.qprop_results='try';
filename.geo = 'a';

cd(dir.work)

%% Trim
% Select aircraft point from the Pareto Front
ii=2
pars=x(ii,:);

%Define the flight conditions to evaluate
TAS_range =[0,15,30,50,70,90,110,125,140,160]/3.6;%True airspeed m/s
Altitude_range=[0]; %altitude m
Vz_range = [0,2,4,6,8,10];%Vertical Airspeed m/s

%With variable propeller tilt angle
tic
for i=1:length(TAS_range)
    for j=1:length(Altitude_range)
        for k = 1:length(Vz_range)-1
            TAS=TAS_range(i);
            Altitude= Altitude_range(j);
            Vz = Vz_range(k);
           % Run trimming routine
            [Trim_sol_Vz_noESDU(i,:,k),Preq_Vz_noESDU(i,k),P_req_forw_Vz_noESDU(i,k),P_req_back_Vz_noESDU(i,k)] = Trimming_routine_No_ESDU_Vz(TAS,Vz,Altitude,pars,pars_BackProp,dir,filename,AerofoilDataset,neural);
        end
    end
end
toc
%% Trim using Back propellers and tilt fixed
%Define the flight conditions to evaluate
tilt_range=[0:10:90];%tilt angle in deg
Trim_sol_Vz_noESDU_tilt_fixed =[];Preq_Vz_noESDU_tilt_fixed=[];
P_req_forw_Vz_noESDU_tilt_fixed=[]; P_req_back_Vz_noESDU_tilt_fixed=[];

j=1;

for i=1:length(TAS_range)
        for k = 1:length(Vz_range)
            for l = 1:length(tilt_range)%1:length(tilt_range)
                tic
                TAS = TAS_range(i);
                Altitude = Altitude_range(j);
                Vz = Vz_range(k);
                tilt = tilt_range(l);
                
                %  condition Vz cannot be higher than TAS
                if TAS >= Vz 
                    [Trim_sol_Vz_noESDU_tilt_fixed(i,:,k,l),Preq_Vz_noESDU_tilt_fixed(i,k,l),...
                        P_req_forw_Vz_noESDU_tilt_fixed(i,k,l),P_req_back_Vz_noESDU_tilt_fixed(i,k,l)] =...
                        Trimming_routine_No_ESDU_Vz_tilt_fixed(TAS,Vz,tilt,Altitude,pars,pars_BackProp,dir,filename,AerofoilDataset,neural);
                else
                    Trim_sol_Vz_noESDU_tilt_fixed(i,:,k,l)=[NaN,NaN,NaN,NaN];
                    Preq_Vz_noESDU_tilt_fixed(i,k,l)=NaN;
                    P_req_forw_Vz_noESDU_tilt_fixed(i,k,l)=NaN;
                    P_req_back_Vz_noESDU_tilt_fixed(i,k,l)=NaN;
                end
                Preq_Vz_noESDU_tilt_fixed
                toc
            end
        end
end

for i=1:length(TAS_range)
        for k = 1:length(Vz_range)
            for l = 1:length(tilt_range)  
                Power_aux(l)=Preq_Vz_noESDU_tilt_fixed(i,k,l);  
            end
            
            [P_sol(i,k),index] = min(Power_aux);
            Trim(i,:,k) = [Trim_sol_Vz_noESDU_tilt_fixed(i,:,k,index),tilt_range(index)];
            P_forw(i,k) = P_req_forw_Vz_noESDU_tilt_fixed(i,k,index);
            P_back(i,k) = P_req_back_Vz_noESDU_tilt_fixed(i,k,index);
            
        end
end

%3D plot Power at each flight condition
surf(Vz_range,TAS_range,P_sol/1000,'FaceColor','interp','EdgeColor','interp')%,['g', 'b','k'])
colormap winter;
cc = colorbar;
cc.Label.String = 'Power(kW)';
xlabel('V_z(m/s)')
ylabel('TAS(m/s)')
zlabel('P(kN)')
title('Power vs true airspeed and vertical airspeed')

set(gcf,'color','w');
% export_fig('TAS and Vz vs P')

%3D plot prop Angle of Attack at each flight condition
 data_plot=[];
gamma=[];
  for i =1:size(TAS_range,2)
      for j =1: size(Vz_range,2)
          TAS = TAS_range(i);
          Vz = Vz_range(j);
          
          if TAS==0
              gamma(i,j) = 0;
          else
              gamma(i,j) = asin(Vz/TAS);
          end
          
          if isreal(Trim(i,2,j)*180/pi + Trim(i,5,j) - gamma(i,j)*180/pi) %&& (Trim(i,2,k)+Trim(i,3,k)-gamma(i,j))>-2 
              data_plot(i,j)= Trim(i,2,j)*180/pi + Trim(i,5,j) - gamma(i,j)*180/pi;
          else
              data_plot(i,j)=NaN;
          end
      end
  end
  
surf(Vz_range,TAS_range,data_plot,'EdgeColor','interp')%,['g', 'b','k'])
colormap winter;
xlabel('V_z(m/s)')
ylabel('TAS(m/s)')
zlabel('\alpha_{prop}(deg)')
title('Propeller angle of attack vs true airspeed and vertical airspeed')
colorbar

set(gcf,'color','w');
