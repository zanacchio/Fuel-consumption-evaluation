function [x_next, stageCost, unfeas, prof] = hev_model(x, u, w, veh)
%hev_model series HEV powertrain model
%   Implements a backward quasi-static model for a series HEV,
%   advancing the simulation by one timestep.
%
% Input arguments
% ---------------
% x : double
%   current value of the state variable(s)
% u : double
%   current value of the control variable(s)
% w : double
%   current value of the exogenous input(s)
% veh : struct
%   structure containing all parameters for the powertrain components
%
% Outputs
% ---------------
% x_next : double
%   value of the state variable(s) at the end of the current timestep
% stageCost : double
%   stage (running) cost incurred
% unfeas : logical
%   when set to true, at least one of the constraints was violated
% prof : struct
%   data structure to return additional quantities of interest for
%   visualization/postprocessing
%
% Model details
% ---------------
% State variables
%   x(1)    Battery SOC, -
% Control variables
%   u(1)    Engine speed, rad/s
%   u(2)    Engine torque, Nm
% Exogenous inputs
%   w(1)    Vehicle speed, m/s
%   w(2)    Vehicle acceleration, m/s^2
% Stage cost
%   stageCost   fuel flow rate, g/s
%
% Usage examples
% ---------------
% Basic usage
%   If SOC, engSpd, engTrq, vehSpd, vehAcc are all scalars, you can find the
%   SOC at the next timestep, fuel consumption, unfeasibilty flag as:
%       [SOC_next, fuelFlwRate, unfeas] = hev_model(SOC, [engSpd, engTrq], [vehSpd, vehAcc], veh)
%   SOC_next, fuelFlwRate and unfeas will be scalars.
%
% Returning additional time profiles
%   Also return the fourth to last outputs:
%       [SOC_next, fuelFlwRate, unfeas, prof] = hev_model( ... )
%   prof will be structures with additional time profiles, (motor power,
%   battery current, ...).
%
%   If you are using this in a for loop, you will probably write something
%   like:
%       [SOC_next(k), fuelFlwRate(k), unfeas(k), prof(k)] = hev_model( ... )
%   In this case, prof will become a non-scalar structure, which you may
%   find hard to manipulate. In this case, convert them to scalar
%   structures containing arrays with structArray2struct:
%       prof = structArray2struct(prof)

%% Driveline (Vehicle + Final Drive + Transmission)
[shaftSpd, demTrq, prof] = hev_drivetrain(w(1), w(2), veh);

%% Motor
% Torque-coupling device (ideal)
motSpd = shaftSpd;
motTrq = demTrq; % Nm

% Electric motor efficiency
motSpd = motSpd.*ones(size(motSpd));
motEff = (shaftSpd~=0) .* veh.mot.effMap(motSpd, motTrq) + (shaftSpd==0);

% Limit Torque
motMaxTrq = veh.mot.maxTrq(motSpd); % Nm
motMinTrq = veh.mot.minTrq(motSpd); % Nm

% Saturate regen braking torque
motTrq = max(motTrq, motMinTrq); % Nm

% Calculate electric power consumption
motElPwr = (motTrq<0) .* motSpd.*motTrq.*motEff + (motTrq>=0) .* motSpd.*motTrq./motEff; % W

% Constraints
motSpdUnfeas = motSpd > veh.mot.maxSpd;
motTrqUnfeas = ( motTrq < motMinTrq ) | ( motTrq > motMaxTrq );
motUnfeas = ( motSpdUnfeas | motTrqUnfeas );

%% Engine
engSpd = u(1); % rad/s
engTrq = u(2); % Nm

% Engine state 
% Assume the engine is turned off when the engine torque is null
engState = engSpd > veh.eng.idleSpd & engTrq > 0;

% Fuel mass flow rate
engSpd = engSpd.*ones(size(engTrq)); % rad/s
fuelFlwRate = veh.eng.fuelMap(engSpd, engTrq); % g/s
fuelFlwRate( engSpd == 0 & engTrq == 0 ) = 0;

% Maximum engine torque
engMaxTrq = veh.eng.maxTrq(engSpd); % Nm

% Constraints
engSpdUnfeas = ( engState == 1 ) & ( ( engSpd < veh.eng.idleSpd ) | ( engSpd > veh.eng.maxSpd ) );
engTrqUnfeas = ( engTrq > engMaxTrq ) | ( engTrq < 0 ); 
engUnfeas =  ( engSpdUnfeas | engTrqUnfeas );

%% Gen
% Torque-coupling device (ideal)
genSpd = engSpd .* veh.gen.tcSpdRatio;
genTrq = - engTrq ./ veh.gen.tcSpdRatio; % Nm

% Generator efficiency
genSpd = genSpd.*ones(size(genTrq));
genEff = (genSpd~=0) .* veh.gen.effMap(genSpd, genTrq) + (genSpd==0);

% Calculate electric power consumption
genElPwr = genSpd.*genTrq.*genEff; % W

% Limit Torque
genMaxTrq = veh.gen.maxTrq(genSpd); % Nm
genMinTrq = veh.gen.minTrq(genSpd); % Nm

% Constraints
genSpdUnfeas = genSpd > veh.gen.maxSpd;
genTrqUnfeas = ( genTrq < genMinTrq ) | ( genTrq > genMaxTrq );
genUnfeas = ( genSpdUnfeas | genTrqUnfeas );

%% Power split
auxPwr = 0;
battPwr = motElPwr + genElPwr + auxPwr;

%% Battery
% Battery internal resistance
battR = veh.batt.eqRes(x(1)); % ohm
% Battery voltage
battOCVolt = veh.batt.ocv(x(1)); % V

% Battery current
battCurr = (battOCVolt-sqrt(battOCVolt.^2 - 4.*battR.*battPwr))./(2.*battR); % A
battCurr = real(battCurr);
% Current limits
maxChrgBattCurr = veh.batt.minCurr; % A
maxDisBattCurr = veh.batt.maxCurr; % A
% Saturate charge current in regen braking
battCurr = max(battCurr, maxChrgBattCurr);

% New battery state of charge
x_next(1)  = - battCurr ./ (veh.batt.nomCap * 3600) .* veh.dt + x(1);
% Constraints
battUnfeas = ( battPwr <= 0 & battCurr < maxChrgBattCurr ) | ( battPwr > 0 & battCurr > maxDisBattCurr );

%% Stage cost
stageCost  = fuelFlwRate;

%% Unfeasibilites
% Combine unfeasibilities
unfeas = ( motUnfeas | engUnfeas | genUnfeas | battUnfeas );

%% Operating mode
opMode = repmat("", size(engState));
opMode( engState == 0 ) = 'pe';
opMode( engState == 1 & battCurr >= 0 ) = 'cd';
opMode( engState == 1 & battCurr < 0 ) = 'bc';

%% Pack additional outputs
prof.fuelFlwRate = fuelFlwRate;
prof.engSpd = engSpd;
prof.engTrq = engTrq;
prof.engSpdUnfeas = engSpdUnfeas;
prof.engTrqUnfeas = engTrqUnfeas;
prof.engineState = engState;

prof.genSpd = genSpd;
prof.genTrq = genTrq;
prof.genElPwr = genElPwr;
prof.genSpdUnfeas = genSpdUnfeas;
prof.genTrqUnfeas = genTrqUnfeas;

prof.motSpd = motSpd;
prof.motTrq = motTrq;
prof.motElPwr = motElPwr;
prof.motSpdUnfeas = motSpdUnfeas;
prof.motTrqUnfeas = motTrqUnfeas;

prof.battSOC = x(1);
prof.battCurr = battCurr;
prof.battVolt = battOCVolt - battR.*battCurr;
prof.battRes = battR;

prof.vehSpd = w(1);
prof.vehAcc = w(2);
prof.opMode = opMode; 

prof.motUnfeas = motUnfeas;
prof.engUnfeas = engUnfeas;
prof.genUnfeas = genUnfeas;
prof.battUnfeas = battUnfeas;
prof.unfeas = unfeas;
