function p = cartpole_params()
%CARTPOLE_PARAMS CP-1 benchmark and numerical parameters.
%
% State order:
%   x(1) = z          cart position [m]
%   x(2) = theta      pole angle from upright [rad]
%   x(3) = z_dot      cart velocity [m/s]
%   x(4) = theta_dot  pole angular velocity [rad/s]
%
% theta = 0 is the upright equilibrium.

% Physical parameters
p.M  = 1.0;       % cart mass [kg]
p.m  = 1.0;       % pole mass [kg]
p.g  = 9.81;      % gravity [m/s^2]
p.l  = 0.5;       % distance pivot -> pendulum COM [m]
p.Kd = 10.0;      % cart viscous damping [N s/m]

% Benchmark specification
p.x0   = [0.20; 0.15; 0.0; 0.0];
p.xref = [0.0; 0.0; 0.0; 0.0];
p.u0   = 0.0;

p.zMin = -2.4;
p.zMax =  2.4;
p.uMin = -100.0;
p.uMax =  100.0;

% Simulation / prediction discretisation
p.Ts = 0.1;              % [s]
p.integrationSubsteps = 4;
p.simulationHorizon = 8.0; % [s]

% NMPC settings
p.predictionHorizon = 20;
p.controlHorizon = 5;

% Output weights in state order [z theta z_dot theta_dot]
p.outputWeights = [8.0 14.0 0.5 1.0];
p.mvWeight = 1e-3;
p.mvRateWeight = 0.10;

% Scale factors improve numerical conditioning
p.outputScale = [2.4 0.30 2.0 3.0];
p.inputScale = 100.0;

% Evaluation scale and tolerance
p.evalScale = [2.4; 0.30; 2.0; 3.0];
p.settlingTolerance = 0.05;  % max scaled state error
end
