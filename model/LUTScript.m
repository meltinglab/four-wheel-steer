function [Klut, Eqlut] = LUTScript(vehicle, maxAngle, resAngle, maxSpeed, resSpeed)

    slotsAngle = 1 + maxAngle / resAngle;
    slotsSpeed = maxSpeed / resSpeed;

    Klut = zeros(slotsAngle, slotsSpeed, 2);
    Eqlut = zeros(slotsAngle, slotsSpeed, 2);
    for a = 2:slotsAngle        % angolo ruote frontali
        for v = 1:slotsSpeed    % velocità
            [Klut(a,v,:), Eqlut(a,v,:)] = LinPlant(a*resAngle, v*resSpeed, vehicle);
        end
    end
    
end