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

load_system('CRReferenceApplication');

switchBlock = find_system('CRReferenceApplication', 'BlockType', 'Constant', 'Name', 'ActivateRearSteering');
%objparams = get_param(switchBlock{1}, 'ObjectParameters');


rngRearSteer = deg2rad(5);              % Rear steering range [rad]

maxAngle = 30;                          % [deg]
resAngle = 3;                           % [deg]
thrAngle = ((resAngle*2)/100) * 10;     % [deg]  <- half sub-range*2 * [%]

maxSpeed = 200;                         % [km/h]
resSpeed = 10;                          % [km/h]
thrSpeed = ((resSpeed*2)/100) * 10;     % [km/h] <- half sub-range*2 * [%]


% Extract vehicle's parameters structure VEH from workspace
VEH = getVariable(get_param(bdroot('CRReferenceApplication'), 'modelworkspace'),'VEH');

% Generate LUTs based on current vehicle parameters
[Klut, Eqlut] = LUTScript(VEH, maxAngle, resAngle, maxSpeed, resSpeed);


% Simulate plain vehicle
set_param(switchBlock{1},'Value','0');
save_system('CRReferenceApplication',[],'SaveDirtyReferencedModels','on');

sim('CRReferenceApplication');

deltaROL = logsout.find('SteerRearCtrl');
tOL = deltaROL.Values.Time(:,1);
deltaROL = deltaROL.Values.Data(:,1);
betaUOL = logsout.find('BetaU');
betaUOL = betaUOL.Values.Data(:,1);
omegaZOL = logsout.find('OmegaZ');
omegaZOL = omegaZOL.Values.Data(:,1);
posXYZOL = logsout.find('PositionXYZ');
posXOL = posXYZOL.Values.Data(:,2);
posYOL = posXYZOL.Values.Data(:,1);

% Simulate with Active Rear Steering
set_param(switchBlock{1},'Value','1');
save_system('CRReferenceApplication',[],'SaveDirtyReferencedModels','on');

sim('CRReferenceApplication');

deltaRRS = logsout.find('SteerRearCtrl');
tRS = deltaRRS.Values.Time(:,1);
deltaRRS = deltaRRS.Values.Data(:,1);
betaREF = logsout.find('RefBeta');
betaREF = betaREF.Values.Data(:,1);
omegaZREF = logsout.find('RefOmgz');
omegaZREF = omegaZREF.Values.Data(:,1);
posXYZRS = logsout.find('PositionXYZ');
posXRS = posXYZRS.Values.Data(:,2);
posYRS = posXYZRS.Values.Data(:,1);

betaURS = logsout.find('BetaU');
betaURS = betaURS.Values.Data(:,1);
omegaZRS = logsout.find('OmegaZ');
omegaZRS = omegaZRS.Values.Data(:,1);


% Plot data
RearSteeringVSOL = figure('Name','Rear Steering vs non Rear Steering','NumberTitle','off')

subplot(2,2,1);
plot(tRS,omegaZRS,'g',tOL, omegaZOL, 'r',tRS,omegaZREF,'c')
title('OmegaZ Comparison')
legend('RS','OL','Ref')

subplot(2,2,2);
plot(tRS,betaURS,'g',tOL, betaUOL, 'r', tRS, betaREF,'c')
title('BetaU Comparison')
legend('RS','OL','Ref')

subplot(2,2,[3,4]);
plot(tRS, deltaRRS,'g',tOL, deltaROL, 'r')
title('DeltaR Comparison')
legend('RS','OL')

TrajectoryComparison = figure('Name','Trajectory Comparison','NumberTitle','off')

plot(posXRS,posYRS,'g', posXOL, posYOL, 'r');
axis equal
legend('RS','OL');