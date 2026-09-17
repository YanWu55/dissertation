function m = cartpole_metrics(result)
%CARTPOLE_METRICS Compute Chapter 5 benchmark metrics for CP-1.

p = result.cfg.p;
t = result.t;
tu = result.tu;
x = result.x;
u = result.u;

e = x - p.xref;
eScaled = e ./ p.evalScale;

% RMS norm of the dimensionless scaled state error
m.NormalizedRMSE = sqrt(mean(sum(eScaled.^2,1)));

errInf = max(abs(eScaled),[],1);
m.SettlingTime = settling_time(t,errInf,p.settlingTolerance);

m.TerminalScaledError = norm(eScaled(:,end),2);
m.MaxPoleAngleDeg = max(abs(rad2deg(x(2,:))));
m.MaxCartPosition = max(abs(x(1,:)));

% Control effort [N^2 s]
m.ControlEffort = trapz(tu,u.^2);
m.PeakInput = max(abs(u));

% Hard-constraint violations
m.MaxStateViolation = max([ ...
    max(p.zMin - x(1,:),0), ...
    max(x(1,:) - p.zMax,0)],[],'all');

m.MaxInputViolation = max([ ...
    max(p.uMin - u,0), ...
    max(u - p.uMax,0)],[],'all');

% Dimensionless closed-loop comparison cost.
uScaled = (u - p.u0)/p.inputScale;
stageState = sum(eScaled(:,1:end-1).^2,1);
stageInput = 0.1*uScaled.^2;
m.ClosedLoopCost = p.Ts*sum(stageState + stageInput);

m.MeanSolveTime = mean(result.solveTime);
m.MaxSolveTime = max(result.solveTime);
m.MeanIterations = mean(result.iterations);
m.SolverSuccessRate = mean(result.exitFlag >= 0);
end
