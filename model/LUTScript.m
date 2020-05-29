function [Klut, Eqlut] = LUTScript(vehicle)

    global resAngle
    global resSpeed
    
    maxAngle = 30;  % [deg]
    resAngle = 3;   % [deg]
    slotsAngle = 1 + maxAngle / resAngle;

    maxSpeed = 200; % [km/h]
    resSpeed = 10;  % [km/h]
    slotsSpeed = maxSpeed / resSpeed;

    Klut = zeros(slotsAngle, slotsSpeed, 2);
    Eqlut = zeros(slotsAngle, slotsSpeed, 2);
    for a = 2:slotsAngle        % angolo ruote frontali
        for v = 1:slotsSpeed    % velocità
            [Klut(a,v,:), Eqlut(a,v,:)] = LinPlant(a*resAngle, v*resSpeed, vehicle);
        end
    end
    
end