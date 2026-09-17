function [nlobj,cfg] = setup_cartpole_nmpc()
%SETUP_CARTPOLE_NMPC Configure the CP-1 nonlinear MPC controller.

p = cartpole_params();

nx = 4;
ny = 4;  % no OutputFcn -> nlmpc assumes y = x
nu = 1;

nlobj = nlmpc(nx,ny,nu);
nlobj.Ts = p.Ts;
nlobj.PredictionHorizon = p.predictionHorizon;
nlobj.ControlHorizon = p.controlHorizon;

nlobj.Model.StateFcn = @cartpole_state_dt;
nlobj.Model.IsContinuousTime = false;

% Standard MPC cost: output tracking + MV + MV-rate penalties
nlobj.Weights.OutputVariables = p.outputWeights;
nlobj.Weights.ManipulatedVariables = p.mvWeight;
nlobj.Weights.ManipulatedVariablesRate = p.mvRateWeight;

% State/output scaling
for i = 1:nx
    nlobj.States(i).ScaleFactor = p.outputScale(i);
    nlobj.OV(i).ScaleFactor = p.outputScale(i);
end
nlobj.MV(1).ScaleFactor = p.inputScale;

% Hard state constraint: finite cart track
nlobj.States(1).Min = p.zMin;
nlobj.States(1).Max = p.zMax;

% Hard actuator constraints
nlobj.MV(1).Min = p.uMin;
nlobj.MV(1).Max = p.uMax;

% Accept a feasible suboptimal result if the solver reaches iteration limit.
nlobj.Optimization.UseSuboptimalSolution = true;

cfg.id = "CP-1";
cfg.p = p;
cfg.x0 = p.x0;
cfg.xref = p.xref;
cfg.yref = p.xref';
cfg.u0 = p.u0;
cfg.Tsim = p.simulationHorizon;
cfg.evalScale = p.evalScale;
cfg.settlingTolerance = p.settlingTolerance;

fprintf('Validating CP-1 NMPC model...\n');
validateFcns(nlobj,cfg.x0,cfg.u0);
end
