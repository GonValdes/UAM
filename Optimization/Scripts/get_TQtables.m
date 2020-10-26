function [Q,rps] = get_TQtables(run,filename,cond,prop)
% Read the data obtained from QPROP

%Input
%run-conditions of V and rpm to be studied
%filename- at least with the data name filename.qprop_results
%cond - at leas with density cond.rho
%prop - geometry, as in file Qprop

%Output
%T = T/(rho * n^2 * d^4)
%Q = Q/(rho * n^2 * d^5)
%T-thrust(N);Q-Torque(N.m) ; rho-density(kg/m3) ; n-rev/s ; d-propeller diameter

try
    DATA = readtable(strcat(filename.qprop_results,'.txt'),'PreserveVariableNames',false);
    A_aux = cell2mat(table2array(DATA(16,2)));
    A = str2num(A_aux(1:55));
    Q = A(5);
    rps = A(2)/60;
catch
    Q = 100000;
    rps = 100000;
end


% for iii=1:length(rpm)%columns
%     for kkk = 1:size(DATA,1)
%         A_aux = cell2mat(table2array(DATA(kkk,1)));
%         if length(A_aux) > 55
%             A = str2num(A_aux(1:55));
%             if (A(2)==rpm(iii))
%                 Q(iii) = A(5);
%                 T(iii) = A(4);
%             end
%         end
%     end
% end



end

