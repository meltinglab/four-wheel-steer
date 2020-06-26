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

function [Klut, Eqlut] = LUTScript(vehicle, maxAngle, resAngle, maxSpeed, resSpeed)

    slotsAngle = 1 + maxAngle / resAngle;
    slotsSpeed = 1 + maxSpeed / resSpeed;

    Klut = zeros(slotsAngle, slotsSpeed, 2);
    Eqlut = zeros(slotsAngle, slotsSpeed, 2);
    for a = 1:slotsAngle
        for v = 1:slotsSpeed
            [Klut(a,v,:), Eqlut(a,v,:)] = LinPlant((a-1)*resAngle, (v-1)*resSpeed, vehicle);
        end
    end

end
