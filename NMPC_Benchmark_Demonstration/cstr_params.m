function p = cstr_params()
%CSTR_PARAMS CSTR-1 benchmark and nonlinear model parameters.
%
% State order:
%   x(1) = C_A  reactant concentration [kmol/m^3]
%   x(2) = T    reactor temperature [K]
%
% Input:
%   u = T_c     coolant temperature [K]

% Standard nonlinear CSTR model parameters
p.F = 1.0;          % volumetric flow rate [m^3/h]
p.V = 1.0;          % reactor volume [m^3]
p.R = 1.985875;     % kcal/(kmol K)
p.dH = -5960.0;     % kcal/kmol
p.E = 11843.0;      % kcal/kmol
p.k0 = 34930800.0;  % 1/h
p.rhoCp = 500.0;    % kcal/(m^3 K)
p.UA = 150.0;       % kcal/(K h)

% Feed conditions
p.CAf = 10.0;       % kmol/m^3
p.Tf = 300.0;       % K

% Initial low-conversion operating point
p.x0 = [8.5698; 311.2639];
p.u0 = 292.0;       % K

% Define the high-conversion target using C_A* = 2.
% Solve the nonlinear steady-state equations exactly for T* and T_c*.
p.CAstar = 2.0;
kRequired = (p.F/p.V)*(p.CAf-p.CAstar)/p.CAstar;
p.Tstar = -p.E/(p.R*log(kRequired/p.k0));
rStar = (p.F/p.V)*(p.CAf-p.CAstar);
alpha = p.UA/(p.rhoCp*p.V);
p.Tcstar = p.Tstar - ( ...
    (p.F/p.V)*(p.Tf-p.Tstar) - (p.dH/p.rhoCp)*rStar )/alpha;

p.xref = [p.CAstar; p.Tstar];

% Benchmark constraints
p.CAMin = 0.0;
p.CAMax = 10.0;
p.TMin = 300.0;
p.TMax = 390.0;
p.TcMin = 273.0;
p.TcMax = 322.0;
p.TcRateMin = -5.0;  % K / control interval
p.TcRateMax =  5.0;

% Simulation / prediction discretisation
p.Ts = 0.5;                  % h
p.integrationSubsteps = 10;
p.simulationHorizon = 20.0;  % h

% NMPC settings
p.predictionHorizon = 12;
p.controlHorizon = 6;

% Outputs are [CA, T].
p.outputWeights = [5.0 2.0];
p.mvWeight = 0.0;        % nonzero nominal coolant temperature
p.mvRateWeight = 0.10;

% Numerical scale factors
p.outputScale = [10.0 100.0];
p.inputScale = 300.0;

% Evaluation scale
p.evalScale = [10.0; 100.0];
p.inputEvalScale = p.TcMax - p.TcMin;
p.settlingTolerance = 0.03;
end
