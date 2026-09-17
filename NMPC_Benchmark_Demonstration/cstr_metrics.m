function m = cstr_metrics(result)
%CSTR_METRICS Compute Chapter 5 benchmark metrics for CSTR-1.

p = result.cfg.p;
t = result.t;
tu = result.tu;
x = result.x;
u = result.u;

e = x - p.xref;
eScaled = e ./ p.evalScale;

m.NormalizedRMSE = sqrt(mean(sum(eScaled.^2,1)));
errInf = max(abs(eScaled),[],1);
m.SettlingTime = settling_time(t,errInf,p.settlingTolerance);

m.CA_RMSE = sqrt(mean((x(1,:) - p.CAstar).^2));
m.T_RMSE = sqrt(mean((x(2,:) - p.Tstar).^2));
m.TerminalScaledError = norm(eScaled(:,end),2);
m.MaxTemperatureDeviation = max(abs(x(2,:) - p.Tstar));

% Control effort relative to the nonzero steady-state input
du = u - p.Tcstar;
m.ControlEffort = trapz(tu,du.^2);
m.PeakInputDeviation = max(abs(du));

% State constraint violations
vCA = max([ ...
    max(p.CAMin - x(1,:),0), ...
    max(x(1,:) - p.CAMax,0)],[],'all');

vT = max([ ...
    max(p.TMin - x(2,:),0), ...
    max(x(2,:) - p.TMax,0)],[],'all');

m.CAConstraintViolation = vCA;
m.TemperatureConstraintViolation = vT;
m.MaxStateViolation = max(vCA,vT);

m.MaxInputViolation = max([ ...
    max(p.TcMin - u,0), ...
    max(u - p.TcMax,0)],[],'all');

% Rate violation relative to the actual applied sequence
duMove = diff([p.u0,u]);
m.MaxInputRateViolation = max([ ...
    max(p.TcRateMin - duMove,0), ...
    max(duMove - p.TcRateMax,0)],[],'all');

% Dimensionless closed-loop comparison cost
uScaled = (u - p.Tcstar)/p.inputEvalScale;
stageState = sum(eScaled(:,1:end-1).^2,1);
stageInput = 0.1*uScaled.^2;
m.ClosedLoopCost = p.Ts*sum(stageState + stageInput);

m.MeanSolveTime = mean(result.solveTime);
m.MaxSolveTime = max(result.solveTime);
m.MeanIterations = mean(result.iterations);
m.SolverSuccessRate = mean(result.exitFlag >= 0);
end
