function [Klut, Eqlut] = LUTScript(vehicle, maxAngle, resAngle, maxSpeed, resSpeed)

    slotsAngle = 1 + maxAngle / resAngle;
    slotsSpeed = 1 + maxSpeed / resSpeed;

    Klut = zeros(slotsAngle, slotsSpeed, 2);
    Eqlut = zeros(slotsAngle, slotsSpeed, 2);
    for a = 2:slotsAngle
        for v = 1:slotsSpeed
            [Klut(a,v,:), Eqlut(a,v,:)] = LinPlant((a-1)*resAngle, (v-1)*resSpeed, vehicle);
        end
    end

end
