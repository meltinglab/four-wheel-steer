% Copyright (C) 2019-2020  Filippo Bellesia, Filippo Catellani, Davide Rocco, Filip Valgimigli
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <https://www.gnu.org/licenses/>.

function [Klqr, xeq] = LinPlant(deltaF, V0, vehicle)

    % Initial conditions
    psi0 = vehicle.InitialYawAngle;
    x0 = vehicle.InitialLongPosition;
    y0 = vehicle.InitialLatPosition;
    deltaF = deg2rad(deltaF);
%   deltaR = 0.1;
    deltaR = 0.0;
    betaU0 = 0;
    %omegaZ0 = 0;

    % Parameters
    g = 9.81;           % Gravity Acceleration [m/s^2]
    rho = 1.225;        % Air Density [kg/m^3]
    S = 2.4;            % Cross Surface [m^2]
    Cx0 = -0.3;         % Drag Coefficient
    Cyb = -0.03;        % Drag Coefficient
    rk = 1;             % Road Type [1-6] (see Partial_mu_long.m)
    m = vehicle.Mass;   % Vehicle Mass [kg]
    r = 0.29;           % Tyre Radius [m]
    CrRaw = 35000;      % Rear Cornering Stiffness (from Simulator)
    CfRaw = 15000;      % Front Cornering Stiffness (from Simulator)

    % Wheel Angles
    delta = [ deltaF deltaF deltaR deltaR ];

    % Wheel displacements with respect to the Center of Mass
    rxf = vehicle.FrontAxlePositionfromCG;      % Front Wheels X Axis Offset
    rxr = -(vehicle.RearAxlePositionfromCG);    % Rear Wheels X Axis Offset
    ryf = vehicle.TrackWidth/2;                 % All Wheels Y Axis Offset
    rz = -(vehicle.HeightCG);                   % All Wheels Z Axis Offset

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
%   Iz = 1200*(rxr^2+ryf^2)^2;  % Z Axis Inertial Matrix
    Iz = vehicle.YawMomentInertia;

    if V0 < 1       % Divide-by-Zero safeguard
        V0 = 1;
    end
    V0 = V0/3.6;    % [km/h] --> [m/s]

    Fwz = m*g/4;
    Fwx = 4*Fwz*(muS*sin(deltaR)+(muL0+CR)*cos(deltaR)-muS*sin(deltaF)+(muL0+CR)*cos(deltaF));
    Fwy = 4*Fwz*(muS*sin(deltaR)+(muL0+CR)*cos(deltaR)+muS*sin(deltaF)+(muL0+CR)*cos(deltaF));

    % Forces system
    dfwzdr = 0;
    dfwzdf = 0;
    dFydbetaU = -muL0*(Fwz*4);
    dFwydr = 2*(dfwzdr*muS*sin(deltaR)+Fwz*muS*cos(deltaR)+dfwzdr*(muL0+CR)*cos(deltaR)+Fwz*(muL0+CR)*sin(deltaR)+dfwzdr*muS*sin(deltaF)+dfwzdr*(muL0+CR)*cos(deltaF));
    dFwydf = 2*(dfwzdf*muS*sin(deltaR)+dfwzdf*(muL0+CR)*cos(deltaR)+dfwzdf*muS*sin(deltaF)+Fwz*muS*cos(deltaF)-Fwz*(muL0+CR)*sin(deltaF)+dfwzdf*(muL0+CR)*cos(deltaF));
    dFwxdr = 2*(dfwzdr*(muL0+CR)*cos(deltaR)-Fwz*(muL0+CR)*sin(deltaR)-dfwzdr*muS*sin(deltaR)-Fwz*muS*cos(deltaR)-dfwzdr*muS*sin(deltaF)+dfwzdr*(muL0+CR)*cos(deltaF));
    dFwxdf = 2*(dfwzdf*(muL0+CR)*cos(deltaR)-dfwzdf*muS*sin(deltaR)+dfwzdf*(muL0+CR)*cos(deltaF)-Fwz*(muL0+CR)*sin(deltaF)-dfwzdf*muS*sin(deltaF)+Fwz*muS*cos(deltaF));

    %a11 = (1/(m*V0))*(-cos(betaU0)*Fwx-sin(betaU0)*Fwy)+(1/(m*V0))*(-sin(betaU0)*(dFwxdbeta)+cos(betaU0)*(dFwydbeta));
    a11 = (1/(m*V0))* dFydbetaU;
    a12 = -1+(1/(m*V0))*(((2*muL0)/V0)*(Fwz*rxf+Fwz*rxr));
    a21 = -V0/Iz*(2*Fwz*muL0*(rxf+rxr)/V0);
    a22 = -1/Iz*(2*Fwz*muL0*(2*(rxf^2 + ryf^2)+2*(rxr^2+ryf^2))/V0);

    A = [a11 a12; a21 a22];

    %System Stability
    eigenvalues = eig(A);
    Re = real(eigenvalues);
    if Re(1)<0 && Re(2)<0
          disp('The matrix A is Hurwitz.');
    end

    b111 = 1/(m*V0)*(-sin(betaU0)*dFwxdr+cos(betaU0)*dFwydr);
    b112 = 1/Iz*(2*CR*Fwz*(rxf)+2*Fwz*muL0*8/7*(rxf));
    B1 = [b111; b112];

    % System Reachability
    Reachable = [B1 A*B1];
    if rank(Reachable) == 2
        disp('The state space is completely reachable.');
    end

    b211 = 1/(m*V0)*(-sin(betaU0)*dFwxdf+cos(betaU0)*dFwydf);
    b212 = 1/Iz*(2*CR*Fwz*(rxr)+2*Fwz*muL0*8/7*(rxr));
    B2 = [b211; b212];

    % Cornering Stiffness system
    lf = sqrt(rxf^2+ryf^2);
    lr = sqrt(rxr^2+ryf^2);

    Cf = muL0*CfRaw;
    Cr = muL0*CrRaw;

    A11 = -(Cf+Cr);
    A12 = -m*V0-(Cf*rxf+Cr*rxr)/(V0);
    A21 = -(Cf*rxf+Cr*rxr);
    A22 = -(Cf*lf^2+Cr*lr^2)/(V0);
    A = [1/(m*V0) 0;0 1/Iz]*[A11 A12; A21 A22];
    B1 = [1/(m*V0) 0;0 1/Iz]*[Cr; Cr*rxr];
    B2 = [1/(m*V0) 0;0 1/Iz]*[Cf; Cf*rxf];

    C = [1 0 ; 0 1];
    D = [0 ; 0];

    xeq = [0;0];
    % xeq = (A)^-1*[-B1*deltaR];
    xeq = (inv(A))*[-B2*(deltaF)];

    % Ideal turn Yaw Rate
    omegaZack = (V0/(rxf-rxr))*tan(deltaF);
    %xeq(2,1) = omegaZack;

    Q = (1/2)*inv([0.005 0; 0 0.02]);       % MAX 1/||x||
    R = inv([0.05]);                  % MAX 1/||u||

    sys = ss(A, B1, C, D);
    sys.StateName = ["betaU","omegaZ"];
    sys.InputName = ["deltar"];

    [Klqr, s, e] = lqr(sys, Q, R, 0);
    Klqr = Klqr';
end
