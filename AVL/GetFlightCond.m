function flight_con = GetFlightCond(altitude,TAS)
% Return flight condition estructure wiht rho,P,T,M at each{altitude,TAS}
% Altitude: vector with altitudes in m
% TAS: Vector with Tas in m/s
for i=1:length(altitude)
    for j=1:length(TAS)
        flight_con{i,j}.Altitude = altitude(i);
        flight_con{i,j}.TAS = TAS(j);
        flight_con{i,j}.P0 = 101325*(1-2.25569*10^-5*altitude(i))^5.2561;
        flight_con{i,j}.T = 288.15 - 6.5 * altitude(i)/1000;
        flight_con{i,j}.rho = flight_con{i,j}.P0/(flight_con{i,j}.T*287); 
        flight_con{i,j}.a = (1.4*287*flight_con{i,j}.T)^0.5;
        flight_con{i,j}.Mach = TAS(j)/flight_con{i,j}.a;
        flight_con{i,j}.Visc = 1.7894*10^(-5)* (flight_con{i,j}.T./288.16).^(3/2)*(288.16+110)./(flight_con{i,j}.T+110);
    end
end
end

