function [info, RearCtrlOut,betaeq,omegazeq, i, j] = optRear(deltaFs,v,beta,omegaz, i, j, Klut, Eqlut, resAngle, resSpeed, maxSpeed, maxAngle, thrAngle, thrSpeed, VEH)

    K   = zeros(2);
    Ref = zeros(2);


    % Select LUT indexes (w/ Schmitt Trigger):
    % If well beyond the previous sub-range, with added guard threshold...
        % ...pick the LUT index centered around the new sub-range.

    deltaF = mean(deltaFs);
    aTrue = rad2deg(abs(deltaF));
    if aTrue > maxAngle
        aTrue = maxAngle;
    end
    aIndex = floor( (aTrue + resAngle/2) / resAngle );
    aQuant = aIndex * resAngle;
    aEdgeHi = i*resAngle + resAngle/2;
    aEdgeLo = i*resAngle - resAngle/2;
    aThHi = (aEdgeHi + thrAngle);
    aThLo = (aEdgeLo - thrAngle);

    if aTrue < aThLo | aTrue > aThHi
        i = aIndex;
    end


    vTrue = abs(v);
    if vTrue > maxSpeed
        vTrue = maxSpeed;
    end
    vIndex = floor( (vTrue + resSpeed/2) / resSpeed );
    vQuant = vIndex * resSpeed;
    vEdgeHi = j*resSpeed + resSpeed/2;
    vEdgeLo = j*resSpeed - resSpeed/2;
    vThHi = (vEdgeHi + thrSpeed);
    vThLo = (vEdgeLo - thrSpeed);

    if vTrue < vThLo | vTrue > vThHi
        j = vIndex;
    end

    info = [aTrue, aIndex, aQuant, aEdgeHi, aEdgeLo, aThHi, aThLo, vTrue, vIndex, vQuant, vEdgeHi, vEdgeLo, vThHi, vThLo];


    % Infer equilibrium point
    CfRaw = 15000;
    CrRaw = 35000;

    muL0 = 0;
    for mu = 1:4
        muL0 = muL0 + Partial_mu_long(mu,0);
    end
    muL0 = muL0/4;
    muL0 = 0.8;

    rxf = VEH.FrontAxlePositionfromCG;      % Front Wheels X Axis Offset (Length)
    rxr = -(VEH.RearAxlePositionfromCG);    % Rear Wheels X Axis Offset (Length)
    Iz = VEH.YawMomentInertia;
    m = VEH.Mass;
    ryf = VEH.TrackWidth/2;
    g = 9.81;

    if (v<1)    % Divide-by-Zero safeguard
        v = 1;
    end
    v = v/3.6;  % [km/h] --> [m/s]

    lf = sqrt(rxf^2+ryf^2);
    lr = sqrt(rxr^2+ryf^2);
    Cf = muL0*CfRaw;
    Cr = muL0*CrRaw;
    A11 = -(Cf+Cr);
    A12 = -m*v-(Cf*rxf+Cr*rxr)/(v);
    A21 = -(Cf*rxf+Cr*rxr);
    A22 = -(Cf*lf^2+Cr*lr^2)/(v);
    A = [1/(m*v) 0;0 1/Iz]*[A11 A12; A21 A22];
    B2 = [1/(m*v) 0;0 1/Iz]*[Cf; Cf*rxf];
    xeq = (inv(A))*[-B2*(deltaF)];
    if abs(xeq(2)) > 0.85*1.3*g/v
        xeq(2) = 0.85*1.3*g/v * sign(xeq(2));
    end
    if abs(xeq(1)) > atan(0.02*1.3*g)
        xeq(1) = atan(0.02*1.3*g) * sign(xeq(1));
    end
    Ref = xeq;
    %omegaZack = (v/(rxf-rxr))*tan(deltaF);
    %xeq(2,1) = omegaZack;

    % Extract from LUTs
    K   =  Klut(i+1, j+1, :);
    %Ref = Eqlut(i+1, j+1, :);

    % Equilibrium references from LUT, matching sign to turn direction
    betaeq   = 0;     %sign(deltaF);
    omegazeq = Ref(2);%sign(deltaF);

    % Control signal output u = -Kx
    RearCtrlOut = -((K(1)*(beta-betaeq)) + (K(2)*(omegaz-omegazeq)));
end
