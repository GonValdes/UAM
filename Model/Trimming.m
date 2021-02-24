Initialization_UAM_v2
load('Trim_cruise.mat')

simsetup.initialXYZ = [0 0 -1000];
simsetup.initialUVW = [50 0 1];
simsetup.initialEuler = [0 0 0]*180/pi ;

simsetup.initialLatLong = [52.0732 -0.620928];% Cranfield airport

%% Get initial data
[ sizes , x0 , names ] = Urban_Air_Mobility_v3
% x0 : initial condition of the state variables.
% names: vector of names of the blocks associated with the model’s states.
% sizes(1) : number of continuous states
% sizes(2) : number of discrete states
% sizes(3) : number of outputs
% sizes(4) : number of inputs
% sizes(5) : not used
% sizes(6) : direct-feedthrough flag (1 = yes, 0 = no)
% sizes(7) : number of sample times
%% Trimming

%Change states initial values for the trimming depending on your conditions
UVW = [78;0;5.06174498171188];%Velocity in body axis
PQR = [0;0;0];%Angular rates in body axis
XYZ_earth = [0;0;-1000];%Position in earth axes
Euler_angles = [0;0.0648033006695734;0];%Euler angles 

x_0trim = [UVW;PQR;...
    -233.056348954480;...%engine lag
    XYZ_earth;...
    -233.056356620888;-5.23023229000660e-35;...%Engines lag 
    Euler_angles;...
    -0.0447500653130391;6.48507001250515e-16;1.33899826536658e-13;-3.76520630858752e-12;...
    1.00486568515921e-12;-2.82563967060057e-11;1.00486568515924e-12;-2.82563967060073e-11;...
    -5.38714885481753e-34;-69.9169071381630;-69.9169043930143;];%Other variables

%Define initial input estimation for the trimming.
%Back propeller rotational speed(rad/s); 
%Forward right prop rotational speed(rad/s);Forward left prop rotationalspeed(rad/s);
%Forward right prop tilt angle(deg);Forward right prop tilt angle(deg);
%Elevator deflection(deg);Aileron deflection(deg);Rudder deflection(deg);

u_0trim = [0;-233;-233;-0.00497;-0.00497;-0.0448;0;0];

%Define other trimming parameters
xfix = zeros(size(x_0trim));xfix([1:2 4:6 10 13])=1; %What states do you want to fix
ufix = zeros(size(u_0trim));ufix([1 7:8]) = 1;%What inputs do you want to fix
yfix = [];%What outputs do you want to fix
yini = [];%Initial oputput estimation
dxini = []; %Initial estimation for the derivative of each state
dxfix = [1:length(u_0trim)];%What states you want to maintain constant during simulation

xfix = [1:2 4:6 10 13]; %What states do you want to fix
ufix = [1 7 8];%What inputs do you want to fix
yfix = [];%What outputs do you want to fix
yini = [];%Initial oputput estimation
dxini = []; %Initial estimation for the derivative of each state
dxfix = [1:sizes(1)];dxfix([8 9]) = [];%What states you want to maintain constant during simulation

%% Trim the model
[xtrim,utrim,ytrim,dxtrim] = trim('Urban_Air_Mobility_v3',x_0trim,u_0trim,yini,...
    xfix,ufix,yfix,dxini,dxfix)

%% Linearize the model

[A,B,C,D]= linmod('Urban_Air_Mobility_v3',xtrim,utrim )
