% Funzione per simulare il comportamento del controllo a  quattro ruote
% sterzanti in varie condizioni, aggiungere una stringa come argomento per
% indicare la condizione scelta:
% - "ConstRadiusSnow" per una curva sulla neve
% - "ConstRadiusWind" per una curva con folata di vento
% - "LaneChange" per un doppio cambio di corsia
% - "LaneChangeIce" per un doppio cambio di corsia con strada ghiacciata
% - "ConstRadius" per una curva semplice

function simRoutine2(condition)

load_system('CRReferenceApplication');

switchBlock = find_system('CRReferenceApplication', 'BlockType', 'Constant', 'Name', 'ActivateRearSteering');

% rngRearSteer = deg2rad(5);              % Rear steering range [rad]
% 
% maxAngle = 30;                          % [deg]
% resAngle = 3;                           % [deg]
% thrAngle = ((resAngle*2)/100) * 10;     % [deg]  <- half sub-range*2 * [%]
% 
% maxSpeed = 200;                         % [km/h]
% resSpeed = 10;                          % [km/h]
% thrSpeed = ((resSpeed*2)/100) * 10;     % [km/h] <- half sub-range*2 * [%]
rngRearSteer = deg2rad(5);
assignin('base','rngRearSteer',rngRearSteer);
maxAngle = 30;
assignin('base','maxAngle',maxAngle);
resAngle = 3;
assignin('base','resAngle',resAngle);
thrAngle = ((resAngle*2)/100) * 10;
assignin('base','thrAngle',thrAngle);
maxSpeed = 200;
assignin('base','maxSpeed',maxSpeed);
resSpeed = 10;
assignin('base','resSpeed',resSpeed);
thrSpeed = ((resSpeed*2)/100) * 10;
assignin('base','thrSpeed',thrSpeed);

% Extract vehicle's parameters structure VEH from workspace
VEH = getVariable(get_param(bdroot('CRReferenceApplication'), 'modelworkspace'),'VEH');

% Generate LUTs based on current vehicle parameters
[Klut, Eqlut] = LUTScript(VEH, maxAngle, resAngle, maxSpeed, resSpeed);
assignin('base','Klut',Klut);
assignin('base','Eqlut',Eqlut);

simroot = bdroot('CRReferenceApplication')

% Constant Radius
switch condition
    case "ConstRadiusSnow"
        set_param([simroot '/Reference Generator/Reference Generator/Custom Maneuver/Custom Maneuver/Maneuver Pattern'],'OverrideUsingVariant','const');
        set_param([simroot '/Environment/Friction'],'OverrideUsingVariant','snow');
        set_param([simroot '/Environment/Wind'],'OverrideUsingVariant','nowind');
    case "ConstRadiusWind"
        set_param([simroot '/Reference Generator/Reference Generator/Custom Maneuver/Custom Maneuver/Maneuver Pattern'],'OverrideUsingVariant','const');
        set_param([simroot '/Environment/Friction'],'OverrideUsingVariant','perfect');
        set_param([simroot '/Environment/Wind'],'OverrideUsingVariant','wind');
    case "LaneChange"
        set_param([simroot '/Reference Generator/Reference Generator/Custom Maneuver/Custom Maneuver/Maneuver Pattern'],'OverrideUsingVariant','lanechange');
        set_param([simroot '/Environment/Friction'],'OverrideUsingVariant','perfect');
        set_param([simroot '/Environment/Wind'],'OverrideUsingVariant','nowind');
    case "LaneChangeIce"
        set_param([simroot '/Reference Generator/Reference Generator/Custom Maneuver/Custom Maneuver/Maneuver Pattern'],'OverrideUsingVariant','lanechange');
        set_param([simroot '/Environment/Friction'],'OverrideUsingVariant','ice');
        set_param([simroot '/Environment/Wind'],'OverrideUsingVariant','nowind');
    case "Doomsday"
        set_param([simroot '/Reference Generator/Reference Generator/Custom Maneuver/Custom Maneuver/Maneuver Pattern'],'OverrideUsingVariant','lanechange');
        set_param([simroot '/Environment/Friction'],'OverrideUsingVariant','ice');
        set_param([simroot '/Environment/Wind'],'OverrideUsingVariant','burst');
    otherwise
        set_param([simroot '/Reference Generator/Reference Generator/Custom Maneuver/Custom Maneuver/Maneuver Pattern'],'OverrideUsingVariant','const');
        set_param([simroot '/Environment/Friction'],'OverrideUsingVariant','perfect');
        set_param([simroot '/Environment/Wind'],'OverrideUsingVariant','nowind');
end
        
% Simulate plain vehicle
set_param(switchBlock{1},'Value','0');
[deltaROL,omegaZOL,omegaZREF,betaUOL,betaREF,posXOL,posYOL,tOL] = RunSimulation(0);

% Simulate with Active Rear Steering
set_param(switchBlock{1},'Value','1');
[deltaRRS,omegaZRS,omegaZREF,betaURS,betaREF,posXRS,posYRS,tRS] = RunSimulation(1);

% Rear steering saturation limits overlay
deltaRMax = zeros(1,size(tRS,1));
for i = 1 : size(tRS,1)
    deltaRMax(1,i) = rngRearSteer;
end

switch condition
    case "ConstRadiusSnow"
        constRadiusSnowStates = figure('Name','Constant Radius with Snow','NumberTitle','off');
        constRadiusSnowTrajectory = figure('Name','Constant Radius with Snow','NumberTitle','off');
        plotDataRSOL(constRadiusSnowStates,constRadiusSnowTrajectory,deltaRRS,omegaZRS,omegaZREF,betaURS,betaREF,posXRS,posYRS,tRS,deltaROL,omegaZOL,betaUOL,posXOL,posYOL,tOL,deltaRMax)
    case "ConstRadiusWind"
        constRadiusWindStates = figure('Name','Constant Radius with Wind Burst','NumberTitle','off');
        constRadiusWindTrajectory = figure('Name','Constant Radius with Wind Burst','NumberTitle','off');
        plotDataRSOL(constRadiusWindStates,constRadiusWindTrajectory,deltaRRS,omegaZRS,omegaZREF,betaURS,betaREF,posXRS,posYRS,tRS,deltaROL,omegaZOL,betaUOL,posXOL,posYOL,tOL,deltaRMax)
    case "LaneChange"
        laneChangeStates = figure('Name','Lane Change','NumberTitle','off');
        laneChangeTrajectory = figure('Name','Lane Change','NumberTitle','off');
        plotDataRSOL(laneChangeStates,laneChangeTrajectory,deltaRRS,omegaZRS,omegaZREF,betaURS,betaREF,posXRS,posYRS,tRS,deltaROL,omegaZOL,betaUOL,posXOL,posYOL,tOL,deltaRMax)
    case "LaneChangeIce"
        laneChangeIceStates = figure('Name','Lane Change with Ice Patch','NumberTitle','off');
        laneChangeIceTrajectory = figure('Name','Lane Change with Ice Patch','NumberTitle','off');
        plotDataRSOL(laneChangeIceStates,laneChangeIceTrajectory,deltaRRS,omegaZRS,omegaZREF,betaURS,betaREF,posXRS,posYRS,tRS,deltaROL,omegaZOL,betaUOL,posXOL,posYOL,tOL,deltaRMax)
    otherwise
        constRadiusStates = figure('Name','Constant Radius','NumberTitle','off');
        constRadiusTrajectory = figure('Name','Constant Radius','NumberTitle','off');
        plotDataRSOL(constRadiusStates,constRadiusTrajectory,deltaRRS,omegaZRS,omegaZREF,betaURS,betaREF,posXRS,posYRS,tRS,deltaROL,omegaZOL,betaUOL,posXOL,posYOL,tOL,deltaRMax)
end

end
function [deltaR,omegaZ,omegaZREF,betaU,betaREF,posX,posY,t] = RunSimulation(rearCTRL)
    
    save_system('CRReferenceApplication',[],'SaveDirtyReferencedModels','on');
    sim('CRReferenceApplication');

    deltaR = logsout.find('SteerRearCtrl');
    t = deltaR.Values.Time(:,1);
    deltaR = deltaR.Values.Data(:,1);    
    posXYZ = logsout.find('PositionXYZ');
    posX = posXYZ.Values.Data(:,2);
    posY = posXYZ.Values.Data(:,1);
    betaU = logsout.find('BetaU');
    betaU = betaU.Values.Data(:,1);
    omegaZ = logsout.find('OmegaZ');
    omegaZ = omegaZ.Values.Data(:,1);
    if rearCTRL == 1
        betaREF = logsout.find('RefBeta');
        betaREF = betaREF.Values.Data(:,1);
        omegaZREF = logsout.find('RefOmgz');
        omegaZREF = omegaZREF.Values.Data(:,1);
    else
        betaREF = zeros(size(t));
        omegaZREF = zeros(size(t));
    end
end
function plotDataRSOL(statesFigure,trajectoryFigure,deltaRRS,omegaZRS,omegaZREF,betaURS,betaREF,posXRS,posYRS,tRS,deltaROL,omegaZOL,betaUOL,posXOL,posYOL,tOL,deltaRMax)
    
    figure(statesFigure);
    subplot(2,2,1);
    plot(tRS,omegaZRS,'g',tOL, omegaZOL, 'r',tRS,omegaZREF,'c')
    title('OmegaZ Comparison')
    legend('RS','OL','Ref')

    subplot(2,2,2);
    plot(tRS,betaURS,'g',tOL, betaUOL, 'r', tRS, betaREF,'c')
    title('BetaU Comparison')
    legend('RS','OL','Ref')

    subplot(2,2,[3,4]);
    plot(tRS, deltaRRS,'g',tOL, deltaROL, 'r', tRS, deltaRMax, 'k', tRS, -deltaRMax, 'k')
    title('DeltaR Comparison')
    legend('RS','OL',"Limits")
    hold off
    
    figure(trajectoryFigure);
    hold on
    plot(posXRS,posYRS,'g', posXOL, posYOL, 'r');
    axis equal
    title('Trajectory Comparison')
    legend('RS','OL');
    hold off
end