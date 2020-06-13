function [Klqr, xeq] = LinPlant(deltaF, V0, vehicle)

    %global m rho S g rxf rxr ryf rx ry rz r Iz
    %global Cx0 Cyb CR rk
    %global delta
    %global xeq

    deg2rad = pi/180;   % Degrees to Radians Coeff.
    rad2deg = 180/pi;   % Radians to Degrees Coeff.

    % Initial conditions
    psi0 = vehicle.InitialYawAngle;
    x0 = vehicle.InitialLongPosition;
    y0 = vehicle.InitialLatPosition;
    deltaF = deg2rad * deltaF;
%    deltaR = 0.1;
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
    rxf = vehicle.FrontAxlePositionfromCG;  % Front Wheels X Axis Offset (Length)
    rxr = -(vehicle.RearAxlePositionfromCG);   % Rear Wheels X Axis Offset (Length)
    ryf = vehicle.TrackWidth/2;                             % All Wheels Y Axis Offset (Width)
    rz = -(vehicle.HeightCG);               % All Wheels Z Axis Offset (Height)

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
%    Iz = 1200*(rxr^2+ryf^2)^2;  % Z Axis Inertial Matrix
    Iz = vehicle.YawMomentInertia;
    V0 = V0/3.6;

    Fwz = m*g/4;
    Fwx = 4*Fwz*(muS*sin(deltaR)+(muL0+CR)*cos(deltaR)-muS*sin(deltaF)+(muL0+CR)*cos(deltaF));
    Fwy = 4*Fwz*(muS*sin(deltaR)+(muL0+CR)*cos(deltaR)+muS*sin(deltaF)+(muL0+CR)*cos(deltaF));

    dfwzdr = 0;
    dfwzdf = 0;
    dFwydr = 2*(dfwzdr*muS*sin(deltaR)+Fwz*muS*cos(deltaR)+dfwzdr*(muL0+CR)*cos(deltaR)+Fwz*(muL0+CR)*sin(deltaR)+dfwzdr*muS*sin(deltaF)+dfwzdr*(muL0+CR)*cos(deltaF));
    dFwydf = 2*(dfwzdf*muS*sin(deltaR)+dfwzdf*(muL0+CR)*cos(deltaR)+dfwzdf*muS*sin(deltaF)+Fwz*muS*cos(deltaF)-Fwz*(muL0+CR)*sin(deltaF)+dfwzdf*(muL0+CR)*cos(deltaF));
    dFwxdr = 2*(dfwzdr*(muL0+CR)*cos(deltaR)-Fwz*(muL0+CR)*sin(deltaR)-dfwzdr*muS*sin(deltaR)-Fwz*muS*cos(deltaR)-dfwzdr*muS*sin(deltaF)+dfwzdr*(muL0+CR)*cos(deltaF));
    dFwxdf = 2*(dfwzdf*(muL0+CR)*cos(deltaR)-dfwzdf*muS*sin(deltaR)+dfwzdf*(muL0+CR)*cos(deltaF)-Fwz*(muL0+CR)*sin(deltaF)-dfwzdf*muS*sin(deltaF)+Fwz*muS*cos(deltaF));

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
        disp('La matrice � raggiungibile!')
    end

    b211 = 1/(m*V0)*(-sin(betaU0)*dFwxdf+cos(betaU0)*dFwydf);
    b212 = 1/Iz*(2*CR*Fwz*(rxr)+2*Fwz*muL0*8/7*(rxr));
    B2 = [b211; b212];

    %Mimmo's interpretation
    lf = sqrt(rxf^2+ryf^2);
    lr = sqrt(rxr^2+ryf^2);
    H = [ones(1,4); ry; -rx];
    FzW = (H'/(H*H'))*([m*g; 0; 0]);

    %Cf = muL0*(FzW(1)+FzW(2));
    Cf = muL0*CfRaw;
    %Cr = muL0*(FzW(3)+FzW(4));
    Cr = muL0*CrRaw;
    A11 = -(Cf+Cr);
    A12 = -m*V0-(Cf*rxf+Cr*rxr)/(V0);
    A21 = -(Cf*rxf+Cr*rxr);
    A22 = -(Cf*lf^2+Cr*lr^2)/(V0);
    A = [1/(m*V0) 0;0 1/Iz]*[A11 A12; A21 A22];
    B1 = [1/(m*V0) 0;0 1/Iz]*[Cr; Cr*rxr];
    B2 = [1/(m*V0) 0;0 1/Iz]*[Cf; Cf*rxf];
    
    s = (Cf*rxf+Cr*rxr)/(Cf+Cr);
    if s > 0
        disp('OVERSTEERED VEHICLE')
        disp(['s = ',num2str(s),' m'])
    elseif s < 0
        disp('UNDERSTEERED VEHICLE')
        disp(['s = ',num2str(s),' m'])
    else
        disp('NEUTRAL STEERED VEHICLE')
    end
    
    C = [1 0 ; 0 1];
    DA = [0 0; 0 0];
    D = [0 ; 0];

    xeq = [0;0];
    % xeq = (A)^-1*[-B1*deltaR];
    xeq = (inv(A))*[-B2*(deltaF)];
    
    % Ideal turn Yaw Rate
     omegaZack = (V0/(rxf-rxr))*tan(deltaF);
    xeq(2,1) = omegaZack;
    
    if abs(xeq(2)) > 0.85*1.3*g/V0
        xeq(2) = 0.85*1.3*g/V0 * sign(xeq(2));
    end
    if abs(xeq(1)) > atan(0.02*1.3*g)
        xeq(1) = atan(0.02*1.3*g) * sign(xeq(1));
    end
    
   
    
    %Q = inv(2*diag([(0.2/180*pi)^2 (1/180*pi)^2]));  % MAX 1/||x||
    %R = inv(4*eye(1)*(40*(2*pi)/60)^2);              % MAX 1/||u||
    Q = inv([0.005 0; 0 0.02]);
    R = inv([0.05]);
    %sysA = ss(A, [B1, B2], C, DA);
    %sysA.StateName = ["betaU","omegaZ"];
    %sysA.InputName = ["deltar","deltaf"];

    sys = ss(A, B1, C, D);
    sys.StateName = ["betaU","omegaZ"];
    sys.InputName = ["deltar"];

    [Klqr, s, e] = lqr(sys, Q, R, 0);
    Klqr = Klqr';
    %res = [Klqr xeq];
    %deltaR = - Klqr*[betaU - xeq(1,1); omegaZ - xeq(1,2); deltaf - xeq(1,3)];
    %ESEMPIO = rad2deg*(-Klqr * [0.1047 - xeq(1,1); 0.15 - xeq(1,2); deltaf - xeq(1,3)])
end