function res = LinPlant(deltaf, V0)

global m rho S g rxf rxr ryf rx ry rz r Iz
global Cx0 Cyb CR rk
global delta
global xeq

deg2rad = pi/180; % Degrees to Radians Coeff.
rad2deg = 180/pi; % Radians to Degrees Coeff.

% Initial conditions
psi0 = 0;
x0 = 0;
y0 = 0;
deltaf = deg2rad * deltaf;
deltar = 0.1;
betaU0 = 0;
omegaZ0 = 0;

% Parameters
g = 9.81;       % Gravity Acceleration [m/s^2]
rho = 1.225;    % Air Density [kg/m^3]
S = 2.4;        % Cross Surface [m^2]
Cx0 = -0.3;     % Drag Coefficient
Cyb = -0.03;    % Drag Coefficient
rk = 1;         % Road Type [1-6] (see Partial_mu_long.m)
m = 1200;       % Vehicle Mass [kg]
r = 0.29;       % Tyre Radius [m]

% Wheel Angles
delta = [ deltaf deltaf deltar deltar ];

% Wheel displacements with respect to the Center of Mass
rxf = 1.4;      % Front Wheels X Axis Offset (Length)
rxr = -1.6;     % Rear Wheels X Axis Offset (Length)
ryf = 0.90;     % All Wheels Y Axis Offset (Width)
rz = -0.70;     % All Wheels Z Axis Offset (Height)

rx = [rxf rxf rxr rxr];
ry = [ryf -ryf -ryf ryf];

muL0 = 0;
for j = 1:4
    muL0 = muL0 + Partial_mu_long(j,0);
end
muL0 = muL0/4;
muS = muL0;

r = 0.28;       % Wheel Radius [m]
CR = -0.05;     % Rolling Resistance
Iz = 1200*(rxr^2+ryf^2)^2;  % Z Axis Inertial Matrix

V0 = V0/3.6;

Fwz = m*g/4;
Fwx = 4*Fwz*(muS*sin(deltar)+(muL0+CR)*cos(deltar)-muS*sin(deltaf)+(muL0+CR)*cos(deltaf));
Fwy = 4*Fwz*(muS*sin(deltar)+(muL0+CR)*cos(deltar)+muS*sin(deltaf)+(muL0+CR)*cos(deltaf));

dfwzdr = 0;
dfwzdf = 0;
dFwydr = 2*(dfwzdr*muS*sin(deltar)+Fwz*muS*cos(deltar)+dfwzdr*(muL0+CR)*cos(deltar)+Fwz*(muL0+CR)*sin(deltar)+dfwzdr*muS*sin(deltaf)+dfwzdr*(muL0+CR)*cos(deltaf));
dFwydf = 2*(dfwzdf*muS*sin(deltar)+dfwzdf*(muL0+CR)*cos(deltar)+dfwzdf*muS*sin(deltaf)+Fwz*muS*cos(deltaf)-Fwz*(muL0+CR)*sin(deltaf)+dfwzdf*(muL0+CR)*cos(deltaf));
dFwxdr = 2*(dfwzdr*(muL0+CR)*cos(deltar)-Fwz*(muL0+CR)*sin(deltar)-dfwzdr*muS*sin(deltar)-Fwz*muS*cos(deltar)-dfwzdr*muS*sin(deltaf)+dfwzdr*(muL0+CR)*cos(deltaf));
dFwxdf = 2*(dfwzdf*(muL0+CR)*cos(deltar)-dfwzdf*muS*sin(deltar)+dfwzdf*(muL0+CR)*cos(deltaf)-Fwz*(muL0+CR)*sin(deltaf)-dfwzdf*muS*sin(deltaf)+Fwz*muS*cos(deltaf));

a11 = (1/(m*V0))*(-cos(betaU0)*Fwx-sin(betaU0)*Fwy);
a12 = -1;
a21 = -V0/Iz*(2*Fwz*muL0*(rxf+rxr)/V0);
a22 = -1/Iz*(2*Fwz*muL0*(2*(rxf^2 + ryf^2)+2*(rxr^2+ryf^2))/V0);

A = [a11 a12; a21 a22];
eig(A)

b111 = 1/(m*V0)*(-sin(betaU0)*dFwxdr+cos(betaU0)*dFwydr);
b112 = 1/Iz*(2*CR*Fwz*(rxf)+2*Fwz*muL0*8/7*(rxf));
B1 = [b111; b112];

Reachable = [B1 A*B1];
if rank(Reachable) == 2
    disp('La matrice è raggiungibile!')
end 

b211 = 1/(m*V0)*(-sin(betaU0)*dFwxdf+cos(betaU0)*dFwydf);
b212 = 1/Iz*(2*CR*Fwz*(rxr)+2*Fwz*muL0*8/7*(rxr));
B2 = [b211; b212];

C = [1 0 ; 0 1];
DA = [0 0; 0 0];
D = [0 ; 0];

xeq = (A)^-1*[-B1*deltar];

%Q = inv(2*diag([(0.2/180*pi)^2 (1/180*pi)^2]));  % MAX 1/||x||
%R = inv(4*eye(1)*(40*(2*pi)/60)^2);              % MAX 1/||u||
Q = inv([0.1 0; 0 0.05]);
R = inv([0.035]);
%sysA = ss(A, [B1, B2], C, DA);
%sysA.StateName = ["betaU","omegaZ"];
%sysA.InputName = ["deltar","deltaf"];

sys = ss(A, B1, C, D);
sys.StateName = ["betaU","omegaZ"];
sys.InputName = ["deltar"];

[Klqr, s, e] = lqr(sys, Q, R, 0);
Klqr = Klqr';
res = [Klqr xeq];
%deltaR = - Klqr*[betaU - xeq(1,1); omegaZ - xeq(1,2); deltaf - xeq(1,3)];
%ESEMPIO = rad2deg*(-Klqr * [0.1047 - xeq(1,1); 0.15 - xeq(1,2); deltaf - xeq(1,3)])