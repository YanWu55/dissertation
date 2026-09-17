function [nlobj,cfg] = setup_cstr_nmpc()
%SETUP_CSTR_NMPC Configure the CSTR-1 nonlinear MPC controller.

p = cstr_params();

nx = 2;
ny = 2;  % no OutputFcn -> y = x
nu = 1;

nlobj = nlmpc(nx,ny,nu);
nlobj.Ts = p.Ts;
nlobj.PredictionHorizon = p.predictionHorizon;
nlobj.ControlHorizon = p.controlHorizon;

nlobj.Model.StateFcn = @cstr_state_dt;
nlobj.Model.IsContinuousTime = false;

nlobj.Weights.OutputVariables = p.outputWeights;
nlobj.Weights.ManipulatedVariables = p.mvWeight;
nlobj.Weights.ManipulatedVariablesRate = p.mvRateWeight;

% Scaling
for i = 1:nx
    nlobj.States(i).ScaleFactor = p.outputScale(i);
    nlobj.OV(i).ScaleFactor = p.outputScale(i);
end
nlobj.MV(1).ScaleFactor = p.inputScale;

% Hard physical state constraints
nlobj.States(1).Min = p.CAMin;
nlobj.States(1).Max = p.CAMax;
nlobj.States(2).Min = p.TMin;
nlobj.States(2).Max = p.TMax;

% Hard manipulated-variable constraints
nlobj.MV(1).Min = p.TcMin;
nlobj.MV(1).Max = p.TcMax;
nlobj.MV(1).RateMin = p.TcRateMin;
nlobj.MV(1).RateMax = p.TcRateMax;

nlobj.Optimization.UseSuboptimalSolution = true;

cfg.id = "CSTR-1";
cfg.p = p;
cfg.x0 = p.x0;
cfg.xref = p.xref;
cfg.yref = p.xref';
cfg.u0 = p.u0;
cfg.Tsim = p.simulationHorizon;
cfg.evalScale = p.evalScale;
cfg.settlingTolerance = p.settlingTolerance;

fprintf('CSTR-1 nonlinear target equilibrium:\n');
fprintf('  C_A* = %.8f kmol/m^3\n',p.CAstar);
fprintf('  T*   = %.8f K\n',p.Tstar);
fprintf('  Tc*  = %.8f K\n',p.Tcstar);

fprintf('Validating CSTR-1 NMPC model...\n');
validateFcns(nlobj,cfg.x0,cfg.u0);
end
