function summaryTable = run_CartPole_control_references()
%RUN_CARTPOLE_CONTROL_REFERENCES
% Reconstructed Cart-Pole LQR + existing NMPC reference generator.
%
% IMPORTANT
% -------------------------------------------------------------------------
% This file uses the ACTUAL CP-1 model/configuration from the supplied
% project files:
%
%   cartpole_params.m
%   cartpole_state_ct.m
%   cartpole_state_dt.m
%   setup_cartpole_nmpc.m
%   cartpole_metrics.m
%   settling_time.m
%
% State order:
%   x = [z; theta; z_dot; theta_dot]
%
% The original historical LQR source file was not available. Therefore the
% LQR below is a NEW, explicitly documented reconstructed design. Do not
% describe its results as reproducing an unrecovered historical LQR run.
%
% Fixed reconstructed continuous-time LQR:
%   Q = diag([10, 150, 1, 5])
%   R = 0.2
%
% The continuous-time plant is numerically linearised at the upright
% equilibrium using cartpole_state_ct, so the LQR model is consistent with
% the supplied nonlinear plant including viscous cart damping.
%
% Appendix A.6.5 cases:
%   Baseline      theta0 = 10 deg
%   Intermediate  theta0 = 30 deg
%   Challenge     theta0 = 40 deg
%
% All other initial states are zero.
%
% Outputs:
%   CartPole_Control_References/
%       CARTPOLE_CTRL_controller_reference.mat
%       CARTPOLE_CTRL_B_reference.mat
%       CARTPOLE_CTRL_I_reference.mat
%       CARTPOLE_CTRL_C_reference.mat
%       CARTPOLE_CTRL_reference_summary.csv
%       CARTPOLE_CTRL_reference_summary.mat

clc;
close all;

%% ------------------------------------------------------------------------
% 0. Required files/toolboxes
% -------------------------------------------------------------------------

requiredFiles = { ...
    'cartpole_params', ...
    'cartpole_state_ct', ...
    'cartpole_state_dt', ...
    'setup_cartpole_nmpc', ...
    'cartpole_metrics', ...
    'settling_time'};

for i = 1:numel(requiredFiles)
    if isempty(which(requiredFiles{i}))
        error('Required function not found on MATLAB path: %s', ...
            requiredFiles{i});
    end
end

if isempty(which('lqr'))
    error(['lqr() was not found. Control System Toolbox is required ', ...
        'for the reconstructed LQR reference.']);
end

if isempty(which('nlmpc'))
    error(['nlmpc was not found. Model Predictive Control Toolbox ', ...
        'is required for the existing NMPC reference.']);
end

fprintf('MATLAB version: %s\n', version);
fprintf('Computer platform: %s\n', computer);

%% ------------------------------------------------------------------------
% 1. Load the actual CP-1 benchmark configuration
% -------------------------------------------------------------------------

p = cartpole_params();

fprintf('\n============================================================\n');
fprintf('Cart-Pole Control Reference Generation\n');
fprintf('============================================================\n');
fprintf('State order: [z theta z_dot theta_dot]\n');
fprintf('M  = %.6f kg\n', p.M);
fprintf('m  = %.6f kg\n', p.m);
fprintf('l  = %.6f m\n', p.l);
fprintf('g  = %.6f m/s^2\n', p.g);
fprintf('Kd = %.6f N s/m\n', p.Kd);
fprintf('Track constraint: [%.3f, %.3f] m\n', p.zMin, p.zMax);
fprintf('Force constraint: [%.3f, %.3f] N\n', p.uMin, p.uMax);
fprintf('Sample time: %.6f s\n', p.Ts);
fprintf('Simulation horizon: %.6f s\n', p.simulationHorizon);

outputFolder = 'CartPole_Control_References';
if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end

%% ------------------------------------------------------------------------
% 2. Reconstructed LQR design
% -------------------------------------------------------------------------

xEq = p.xref(:);
uEq = p.u0;

[A,B] = localNumericalLinearization(xEq,uEq);

controllabilityRank = rank(ctrb(A,B));

Q = diag([10, 150, 1, 5]);
R = 0.2;

[K,P,lqrEigenvalues] = lqr(A,B,Q,R);
Acl = A - B*K;
isHurwitz = all(real(lqrEigenvalues) < 0);

careResidual = A'*P + P*A - P*B*(R\(B'*P)) + Q;
careResidualNorm = norm(careResidual,'fro');

fprintf('\n============================================================\n');
fprintf('Reconstructed LQR controller\n');
fprintf('============================================================\n');
fprintf('Controllability rank: %d / 4\n', controllabilityRank);
fprintf('Q = diag([10 150 1 5])\n');
fprintf('R = 0.2\n');
fprintf('\nK =\n');
disp(K);
fprintf('Closed-loop eigenvalues:\n');
disp(lqrEigenvalues);
fprintf('A-BK Hurwitz: %d\n', isHurwitz);
fprintf('CARE residual Frobenius norm: %.6e\n', careResidualNorm);

if controllabilityRank < 4
    error('Linearised CP-1 model is not controllable.');
end

if ~isHurwitz
    error('Reconstructed LQR closed-loop linearisation is not Hurwitz.');
end

controllerMetadata.system = 'Cart-Pole';
controllerMetadata.benchmarkFamily = 'Constrained Upright Stabilisation';
controllerMetadata.stateOrder = 'z theta z_dot theta_dot';
controllerMetadata.lqrStatus = ...
    'Reconstructed fixed LQR design; original historical LQR file unavailable';
controllerMetadata.Q = Q;
controllerMetadata.R = R;
controllerMetadata.K = K;
controllerMetadata.P = P;
controllerMetadata.A = A;
controllerMetadata.B = B;
controllerMetadata.closedLoopEigenvalues = lqrEigenvalues;
controllerMetadata.controllabilityRank = controllabilityRank;
controllerMetadata.careResidualNorm = careResidualNorm;
controllerMetadata.matlabVersion = version;
controllerMetadata.computer = computer;
controllerMetadata.dateGenerated = char(datetime('now'));

save(fullfile(outputFolder,'CARTPOLE_CTRL_controller_reference.mat'), ...
    'A','B','Q','R','K','P','Acl','lqrEigenvalues', ...
    'controllerMetadata','p');

%% ------------------------------------------------------------------------
% 3. Appendix B/I/C instances
% -------------------------------------------------------------------------

cases(1).label = 'B';
cases(1).difficulty = 'Baseline';
cases(1).theta0_deg = 10;

cases(2).label = 'I';
cases(2).difficulty = 'Intermediate';
cases(2).theta0_deg = 30;

cases(3).label = 'C';
cases(3).difficulty = 'Challenge';
cases(3).theta0_deg = 40;

% Summary storage
Difficulty = strings(3,1);
Initial_Angle_deg = zeros(3,1);

LQR_Success = false(3,1);
LQR_Settling_Time_s = nan(3,1);
LQR_Max_Cart_Position_m = nan(3,1);
LQR_Max_Pole_Angle_deg = nan(3,1);
LQR_Peak_Input_N = nan(3,1);
LQR_Control_Effort_N2s = nan(3,1);
LQR_Closed_Loop_Cost = nan(3,1);
LQR_Mean_Compute_Time_s = nan(3,1);

NMPC_Success = false(3,1);
NMPC_Settling_Time_s = nan(3,1);
NMPC_Max_Cart_Position_m = nan(3,1);
NMPC_Max_Pole_Angle_deg = nan(3,1);
NMPC_Peak_Input_N = nan(3,1);
NMPC_Control_Effort_N2s = nan(3,1);
NMPC_Closed_Loop_Cost = nan(3,1);
NMPC_Mean_Solve_Time_s = nan(3,1);
NMPC_Solver_Success_Rate = nan(3,1);

%% ------------------------------------------------------------------------
% 4. Run all three reference cases
% -------------------------------------------------------------------------

for kCase = 1:numel(cases)

    label = cases(kCase).label;
    difficulty = cases(kCase).difficulty;
    theta0_deg = cases(kCase).theta0_deg;

    x0 = [0; deg2rad(theta0_deg); 0; 0];

    fprintf('\n============================================================\n');
    fprintf('Running Cart-Pole Control Reference: %s\n', difficulty);
    fprintf('Initial pole angle: %.1f deg\n', theta0_deg);
    fprintf('============================================================\n');

    %% LQR
    fprintf('\nRunning reconstructed saturated LQR...\n');
    lqrResult = localRunLQR(x0,K,p);
    lqrSuccess = localReferenceSuccess(lqrResult);

    fprintf('LQR success: %d\n', lqrSuccess);
    fprintf('LQR settling time: %.6f s\n', lqrResult.metrics.SettlingTime);
    fprintf('LQR max |z|: %.6f m\n', lqrResult.metrics.MaxCartPosition);
    fprintf('LQR max |theta|: %.6f deg\n', lqrResult.metrics.MaxPoleAngleDeg);
    fprintf('LQR peak |u|: %.6f N\n', lqrResult.metrics.PeakInput);
    fprintf('LQR control effort: %.6f N^2 s\n', ...
        lqrResult.metrics.ControlEffort);

    %% NMPC
    fprintf('\nRunning existing CP-1 NMPC...\n');
    nmpcResult = localRunNMPC(x0);
    nmpcSuccess = localReferenceSuccess(nmpcResult);

    fprintf('NMPC success: %d\n', nmpcSuccess);
    fprintf('NMPC settling time: %.6f s\n', nmpcResult.metrics.SettlingTime);
    fprintf('NMPC max |z|: %.6f m\n', nmpcResult.metrics.MaxCartPosition);
    fprintf('NMPC max |theta|: %.6f deg\n', nmpcResult.metrics.MaxPoleAngleDeg);
    fprintf('NMPC peak |u|: %.6f N\n', nmpcResult.metrics.PeakInput);
    fprintf('NMPC control effort: %.6f N^2 s\n', ...
        nmpcResult.metrics.ControlEffort);

    %% Reference metadata
    metadata.system = 'Cart-Pole';
    metadata.task = 'Control Design';
    metadata.benchmarkFamily = 'Constrained Upright Stabilisation';
    metadata.difficulty = difficulty;
    metadata.referenceType = 'Analytical + Numerical';
    metadata.initialState = x0;
    metadata.initialPoleAngle_deg = theta0_deg;
    metadata.stateOrder = 'z theta z_dot theta_dot';

    metadata.cartMass_kg = p.M;
    metadata.poleMass_kg = p.m;
    metadata.poleLength_m = p.l;
    metadata.gravity_mps2 = p.g;
    metadata.cartDamping_Ns_per_m = p.Kd;

    metadata.cartPositionMin_m = p.zMin;
    metadata.cartPositionMax_m = p.zMax;
    metadata.forceMin_N = p.uMin;
    metadata.forceMax_N = p.uMax;

    metadata.sampleTime_s = p.Ts;
    metadata.simulationHorizon_s = p.simulationHorizon;

    metadata.lqrStatus = ...
        'Reconstructed fixed LQR; original historical LQR source unavailable';
    metadata.lqrQ = Q;
    metadata.lqrR = R;
    metadata.lqrK = K;
    metadata.lqrSuccess = lqrSuccess;

    metadata.nmpcPredictionHorizon = p.predictionHorizon;
    metadata.nmpcControlHorizon = p.controlHorizon;
    metadata.nmpcSuccess = nmpcSuccess;

    metadata.matlabVersion = version;
    metadata.computer = computer;
    metadata.dateGenerated = char(datetime('now'));

    referenceFile = fullfile(outputFolder, ...
        sprintf('CARTPOLE_CTRL_%s_reference.mat',label));

    save(referenceFile, ...
        'lqrResult','nmpcResult','metadata', ...
        'A','B','Q','R','K','P','p','-v7.3');

    fprintf('\nSaved reference file:\n%s\n', referenceFile);

    %% Summary
    Difficulty(kCase) = string(difficulty);
    Initial_Angle_deg(kCase) = theta0_deg;

    LQR_Success(kCase) = lqrSuccess;
    LQR_Settling_Time_s(kCase) = lqrResult.metrics.SettlingTime;
    LQR_Max_Cart_Position_m(kCase) = lqrResult.metrics.MaxCartPosition;
    LQR_Max_Pole_Angle_deg(kCase) = lqrResult.metrics.MaxPoleAngleDeg;
    LQR_Peak_Input_N(kCase) = lqrResult.metrics.PeakInput;
    LQR_Control_Effort_N2s(kCase) = lqrResult.metrics.ControlEffort;
    LQR_Closed_Loop_Cost(kCase) = lqrResult.metrics.ClosedLoopCost;
    LQR_Mean_Compute_Time_s(kCase) = lqrResult.metrics.MeanSolveTime;

    NMPC_Success(kCase) = nmpcSuccess;
    NMPC_Settling_Time_s(kCase) = nmpcResult.metrics.SettlingTime;
    NMPC_Max_Cart_Position_m(kCase) = nmpcResult.metrics.MaxCartPosition;
    NMPC_Max_Pole_Angle_deg(kCase) = nmpcResult.metrics.MaxPoleAngleDeg;
    NMPC_Peak_Input_N(kCase) = nmpcResult.metrics.PeakInput;
    NMPC_Control_Effort_N2s(kCase) = nmpcResult.metrics.ControlEffort;
    NMPC_Closed_Loop_Cost(kCase) = nmpcResult.metrics.ClosedLoopCost;
    NMPC_Mean_Solve_Time_s(kCase) = nmpcResult.metrics.MeanSolveTime;
    NMPC_Solver_Success_Rate(kCase) = ...
        nmpcResult.metrics.SolverSuccessRate;
end

%% ------------------------------------------------------------------------
% 5. Save summary
% -------------------------------------------------------------------------

summaryTable = table( ...
    Difficulty, ...
    Initial_Angle_deg, ...
    LQR_Success, ...
    LQR_Settling_Time_s, ...
    LQR_Max_Cart_Position_m, ...
    LQR_Max_Pole_Angle_deg, ...
    LQR_Peak_Input_N, ...
    LQR_Control_Effort_N2s, ...
    LQR_Closed_Loop_Cost, ...
    LQR_Mean_Compute_Time_s, ...
    NMPC_Success, ...
    NMPC_Settling_Time_s, ...
    NMPC_Max_Cart_Position_m, ...
    NMPC_Max_Pole_Angle_deg, ...
    NMPC_Peak_Input_N, ...
    NMPC_Control_Effort_N2s, ...
    NMPC_Closed_Loop_Cost, ...
    NMPC_Mean_Solve_Time_s, ...
    NMPC_Solver_Success_Rate);

fprintf('\n============================================================\n');
fprintf('REFERENCE SUMMARY\n');
fprintf('============================================================\n');
disp(summaryTable);

writetable(summaryTable, ...
    fullfile(outputFolder,'CARTPOLE_CTRL_reference_summary.csv'));

save(fullfile(outputFolder,'CARTPOLE_CTRL_reference_summary.mat'), ...
    'summaryTable','A','B','Q','R','K','P');

fprintf('\n============================================================\n');
fprintf('All Cart-Pole control references completed.\n');
fprintf('============================================================\n');
fprintf('Output folder:\n%s\n', fullfile(pwd,outputFolder));

end


%% =========================================================================
% LOCAL FUNCTION: numerical continuous-time linearisation
% =========================================================================

function [A,B] = localNumericalLinearization(xEq,uEq)

nx = numel(xEq);

A = zeros(nx,nx);
B = zeros(nx,1);

epsX = 1e-6;
epsU = 1e-6;

for i = 1:nx
    dx = zeros(nx,1);
    dx(i) = epsX;

    fp = cartpole_state_ct(xEq + dx,uEq);
    fm = cartpole_state_ct(xEq - dx,uEq);

    A(:,i) = (fp - fm)/(2*epsX);
end

fp = cartpole_state_ct(xEq,uEq + epsU);
fm = cartpole_state_ct(xEq,uEq - epsU);

B(:,1) = (fp - fm)/(2*epsU);

end


%% =========================================================================
% LOCAL FUNCTION: reconstructed saturated LQR nonlinear simulation
% =========================================================================

function result = localRunLQR(x0,K,p)

N = round(p.simulationHorizon/p.Ts);
t = (0:N)*p.Ts;

x = zeros(4,N+1);
u = zeros(1,N);
solveTime = zeros(1,N);

x(:,1) = x0;

for k = 1:N
    timer = tic;

    e = x(:,k) - p.xref;
    uRaw = p.u0 - K*e;

    uk = min(max(uRaw,p.uMin),p.uMax);

    solveTime(k) = toc(timer);

    u(k) = uk;
    x(:,k+1) = cartpole_state_dt(x(:,k),uk);
end

cfg.id = "CP-1-LQR-Reconstructed";
cfg.p = p;
cfg.x0 = x0;
cfg.xref = p.xref;
cfg.yref = p.xref';
cfg.u0 = p.u0;
cfg.Tsim = p.simulationHorizon;
cfg.evalScale = p.evalScale;
cfg.settlingTolerance = p.settlingTolerance;

result.id = cfg.id;
result.t = t;
result.tu = t(1:end-1);
result.x = x;
result.u = u;
result.solveTime = solveTime;
result.exitFlag = ones(1,N);
result.iterations = zeros(1,N);
result.predictedCost = nan(1,N);
result.cfg = cfg;

result.metrics = cartpole_metrics(result);

end


%% =========================================================================
% LOCAL FUNCTION: existing CP-1 NMPC with custom initial condition
% =========================================================================

function result = localRunNMPC(x0)

[nlobj,cfg] = setup_cartpole_nmpc();
p = cfg.p;

cfg.x0 = x0;

% Validate the same NMPC model for the requested reference initial state.
validateFcns(nlobj,cfg.x0,cfg.u0);

N = round(cfg.Tsim/p.Ts);
t = (0:N)*p.Ts;

x = zeros(4,N+1);
u = zeros(1,N);
solveTime = zeros(1,N);
exitFlag = zeros(1,N);
iterations = zeros(1,N);
predictedCost = nan(1,N);

x(:,1) = cfg.x0;
lastMV = cfg.u0;

opt = nlmpcmoveopt;

for k = 1:N

    timer = tic;
    [mv,opt,info] = ...
        nlmpcmove(nlobj,x(:,k),lastMV,cfg.yref,[],opt);
    solveTime(k) = toc(timer);

    u(k) = mv(1);
    exitFlag(k) = info.ExitFlag;
    iterations(k) = info.Iterations;

    if info.ExitFlag >= 0
        predictedCost(k) = info.Cost;
    end

    if info.ExitFlag < 0
        warning('CP-1:SolverFailure', ...
            ['NMPC solver returned ExitFlag=%d at t=%.3f s. ', ...
             'Returned MV is propagated, matching the supplied ', ...
             'reference implementation behaviour.'], ...
            info.ExitFlag,t(k));
    end

    x(:,k+1) = cartpole_state_dt(x(:,k),mv);
    lastMV = mv;
end

result.id = cfg.id;
result.t = t;
result.tu = t(1:end-1);
result.x = x;
result.u = u;
result.solveTime = solveTime;
result.exitFlag = exitFlag;
result.iterations = iterations;
result.predictedCost = predictedCost;
result.cfg = cfg;

result.metrics = cartpole_metrics(result);

end


%% =========================================================================
% LOCAL FUNCTION: common benchmark success interpretation
% =========================================================================

function success = localReferenceSuccess(result)

m = result.metrics;

success = ...
    ~isnan(m.SettlingTime) && ...
    m.MaxStateViolation <= 1e-9 && ...
    m.MaxInputViolation <= 1e-9 && ...
    m.SolverSuccessRate >= 0.999999;

end
