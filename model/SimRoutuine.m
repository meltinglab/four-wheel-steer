load_system('CRReferenceApplication');

switchBlock = find_system('CRReferenceApplication', 'BlockType', 'Constant', 'Name', 'ActivateRearSteering');
%objparams = get_param(switchBlock{1}, 'ObjectParameters');

% Extract vehicle's parameters structure VEH from workspace
VEH = getVariable(get_param(bdroot('CRReferenceApplication'), 'modelworkspace'),'VEH');

% Generate LUTs based on current vehicle parameters
[Klut, Eqlut] = LUTScript(VEH);

rngRearSteer = (5/180)*pi;    % Rear steering range [rad]

% Simulate with Active Rear Steering
set_param(switchBlock{1},'Value','1');
save_system('CRReferenceApplication',[],'SaveDirtyReferencedModels','on');
sim('CRReferenceApplication');

deltaRRS = logsout.find('SteerRearCtrl');
tRS = deltaRRS.Values.Time(:,1);
deltaRRS = deltaRRS.Values.Data(:,1);
betaURS = logsout.find('betaU');
betaURS = betaURS.Values.Data(:,1);
omegaZRS = logsout.find('BodyAngleDerivatives');
omegaZRS = omegaZRS.Values.Data(:,3);

% Simulate plain vehicle
set_param(switchBlock{1},'Value','0');
save_system('CRReferenceApplication',[],'SaveDirtyReferencedModels','on');

sim('CRReferenceApplication');

deltaROL = logsout.find('betaU');
tOL = deltaROL.Values.Time(:,1);
deltaROL = deltaROL.Values.Data(:,1);
betaUOL = logsout.find('betaU');
betaUOL = betaUOL.Values.Data(:,1);
omegaZOL = logsout.find('BodyAngleDerivatives');
omegaZOL = omegaZOL.Values.Data(:,3);


RearSteeringVSOL = figure('Name','Rear Steering vs non Rear Steering','NumberTitle','off')

subplot(2,2,1);
plot(tRS,omegaZRS,'g',tOL, omegaZOL, 'r')
title('OmegaZ Comparison')
legend('RS','OL')

subplot(2,2,2);
plot(tRS,betaURS,'g',tOL, betaUOL, 'r')
title('betaU Comparison')
legend('RS','OL')

subplot(2,2,[3,4]);
plot(tRS, deltaRRS,'g',tOL, deltaROL, 'r')
title('deltaR Comparison')
legend('RS','OL')