function summaryTable = run_CSTR_control_references()
%RUN_CSTR_CONTROL_REFERENCES
% A.10 CSTR — Constrained Regulation
%
% Target environment:
%   MATLAB R2026a
%   Model Predictive Control Toolbox
%   Optimization Toolbox
%
% This reference generator is built directly on the existing CSTR-1 files:
%   cstr_params.m
%   cstr_state_ct.m
%   cstr_state_dt.m
%   setup_cstr_nmpc.m
%   cstr_metrics.m
%   settling_time.m
%
% Benchmark family:
%   Control Design — Constrained Regulation
%
% Benchmark role:
%   Structural Challenge
%
% Reference type:
%   Analytical + Numerical
%
% Fixed CSTR-1 target:
%   C_A* = 2.0 kmol/m^3
%   T*   = computed exactly from the nonlinear steady-state equations
%   Tc*  = computed exactly from the nonlinear steady-state equations
%
% Three benchmark instances:
%
%   x0(alpha) = x* + alpha (x_LC - x*)
%   Tc0(alpha)= Tc* + alpha (292 - Tc*)
%
%   Baseline:      alpha = 0.25
%   Intermediate:  alpha = 0.50
%   Challenge:     alpha = 1.00
%
% The Challenge case is therefore exactly the original CSTR-1
% low-conversion-to-high-conversion regulation problem.
%
% Fixed constraints:
%   0   <= C_A <= 10 kmol/m^3
%   300 <= T   <= 390 K
%   273 <= Tc  <= 322 K
%   -5  <= Delta Tc <= 5 K/control interval
%
% Fixed NMPC:
%   Ts = 0.5 h
%   Prediction horizon = 12
%   Control horizon = 6
%   Output weights = [5 2]
%   MV rate weight = 0.10
%
% Success criterion:
%   1. Settling time is achieved within the 20 h horizon using the
%      existing CSTR-1 scaled infinity-norm tolerance of 0.03.
%   2. No state, input, or input-rate constraint violation occurs.
%   3. Every NMPC solve returns ExitFlag >= 0.
%
% Independent numerical consistency check:
%   The applied piecewise-constant coolant sequence is replayed through
%   the continuous-time CSTR model using ode113. This checks the RK4
%   discretisation used by the NMPC benchmark. It is NOT a second
%   closed-loop controller run because the control sequence is not
%   recomputed on the ode113 replay trajectory.
%
% Generated files:
%   CSTR_Control_References/
%       CSTR_CTRL_configuration_reference.mat
%       CSTR_CTRL_B_reference.mat
%       CSTR_CTRL_I_reference.mat
%       CSTR_CTRL_C_reference.mat
%       CSTR_CTRL_B_trajectory.csv
%       CSTR_CTRL_I_trajectory.csv
%       CSTR_CTRL_C_trajectory.csv
%       CSTR_CTRL_reference_summary.csv
%       CSTR_CTRL_reference_summary.mat
%
% IMPORTANT DISSERTATION WORDING:
%   The closed-loop simulations demonstrate constraint satisfaction for
%   the prescribed benchmark instances. They are not formal verification
%   proofs for uncertain initial sets.

clc;
close all;

%% ========================================================================
% 0. User controls
% =========================================================================

% Run all official benchmark instances.
% For a quick test, use for example:
% runLabels = ["B"];
runLabels = ["B","I","C"];

% Independent continuous-time replay validation.
runOde113Replay = true;

%% ========================================================================
% 1. Environment and dependency checks
% =========================================================================

fprintf('MATLAB version: %s\n', version);
fprintf('Computer platform: %s\n', computer);

requiredFiles = { ...
    'cstr_params', ...
    'cstr_state_ct', ...
    'cstr_state_dt', ...
    'setup_cstr_nmpc', ...
    'cstr_metrics', ...
    'settling_time'};

for i = 1:numel(requiredFiles)
    if isempty(which(requiredFiles{i}))
        error('Required CSTR-1 file not found on MATLAB path: %s.m', ...
            requiredFiles{i});
    end
end

if exist('check_requirements','file') ~= 0
    check_requirements();
else
    if exist('nlmpc','file') == 0
        error(['nlmpc() was not found. Model Predictive Control Toolbox ', ...
               'is required.']);
    end
end

%% ========================================================================
% 2. Load the fixed shared CSTR-1 plant and NMPC configuration
% =========================================================================

p = cstr_params();

[nlobj,cfgBase] = setup_cstr_nmpc();

% Analytical target consistency check.
targetResidual = cstr_state_ct(p.xref,p.Tcstar);
targetResidualInf = norm(targetResidual,inf);

fprintf('\n============================================================\n');
fprintf('A.10 CSTR — Constrained Regulation\n');
fprintf('============================================================\n');

fprintf('State order: [C_A T]\n');
fprintf('Input: coolant temperature T_c\n');

fprintf('\nFixed nonlinear target equilibrium:\n');
fprintf('C_A*  = %.10f kmol/m^3\n',p.CAstar);
fprintf('T*    = %.10f K\n',p.Tstar);
fprintf('Tc*   = %.10f K\n',p.Tcstar);
fprintf('Equilibrium residual ||f(x*,u*)||_inf = %.6e\n', ...
    targetResidualInf);

fprintf('\nOriginal low-conversion operating point:\n');
fprintf('C_A0  = %.10f kmol/m^3\n',p.x0(1));
fprintf('T0    = %.10f K\n',p.x0(2));
fprintf('Tc0   = %.10f K\n',p.u0);

fprintf('\nPhysical constraints:\n');
fprintf('C_A in [%.3f, %.3f] kmol/m^3\n',p.CAMin,p.CAMax);
fprintf('T   in [%.3f, %.3f] K\n',p.TMin,p.TMax);
fprintf('Tc  in [%.3f, %.3f] K\n',p.TcMin,p.TcMax);
fprintf('Delta Tc in [%.3f, %.3f] K/control interval\n', ...
    p.TcRateMin,p.TcRateMax);

fprintf('\nFixed NMPC configuration:\n');
fprintf('Sampling time: %.6f h\n',p.Ts);
fprintf('RK4 substeps per sample: %d\n',p.integrationSubsteps);
fprintf('Prediction horizon: %d\n',p.predictionHorizon);
fprintf('Control horizon: %d\n',p.controlHorizon);
fprintf('Simulation horizon: %.6f h\n',p.simulationHorizon);
fprintf('Settling tolerance (scaled infinity norm): %.6f\n', ...
    p.settlingTolerance);

% Raw target-neighbourhood interpretation.
targetCATolerance = p.settlingTolerance*p.evalScale(1);
targetTTolerance  = p.settlingTolerance*p.evalScale(2);

fprintf('Equivalent target neighbourhood: |C_A-C_A*| <= %.6f kmol/m^3, ', ...
    targetCATolerance);
fprintf('|T-T*| <= %.6f K\n',targetTTolerance);

%% ========================================================================
% 3. Output folder
% =========================================================================

outputFolder = 'CSTR_Control_References';

if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end

%% ========================================================================
% 4. Save fixed benchmark / controller configuration
% =========================================================================

configurationMetadata.system = 'CSTR';
configurationMetadata.id = 'CSTR-1';
configurationMetadata.task = 'Control Design';
configurationMetadata.benchmarkFamily = 'Constrained Regulation';
configurationMetadata.benchmarkRole = 'Structural Challenge';
configurationMetadata.referenceType = 'Analytical + Numerical';

configurationMetadata.stateOrder = 'C_A T';
configurationMetadata.input = 'T_c';

configurationMetadata.targetState = p.xref;
configurationMetadata.targetInput = p.Tcstar;
configurationMetadata.targetResidualInf = targetResidualInf;

configurationMetadata.CAMin = p.CAMin;
configurationMetadata.CAMax = p.CAMax;
configurationMetadata.TMin = p.TMin;
configurationMetadata.TMax = p.TMax;
configurationMetadata.TcMin = p.TcMin;
configurationMetadata.TcMax = p.TcMax;
configurationMetadata.TcRateMin = p.TcRateMin;
configurationMetadata.TcRateMax = p.TcRateMax;

configurationMetadata.Ts_h = p.Ts;
configurationMetadata.integrationSubsteps = p.integrationSubsteps;
configurationMetadata.predictionHorizon = p.predictionHorizon;
configurationMetadata.controlHorizon = p.controlHorizon;
configurationMetadata.simulationHorizon_h = p.simulationHorizon;

configurationMetadata.outputWeights = p.outputWeights;
configurationMetadata.mvWeight = p.mvWeight;
configurationMetadata.mvRateWeight = p.mvRateWeight;

configurationMetadata.outputScale = p.outputScale;
configurationMetadata.inputScale = p.inputScale;
configurationMetadata.evalScale = p.evalScale;
configurationMetadata.settlingTolerance = p.settlingTolerance;

configurationMetadata.targetCATolerance = targetCATolerance;
configurationMetadata.targetTTolerance = targetTTolerance;

configurationMetadata.matlabVersion = version;
configurationMetadata.computer = computer;
configurationMetadata.dateGenerated = char(datetime('now'));

save( ...
    fullfile(outputFolder,'CSTR_CTRL_configuration_reference.mat'), ...
    'p','nlobj','cfgBase','configurationMetadata','-v7.3');

%% ========================================================================
% 5. Define the three official benchmark instances
% =========================================================================

xLowConversion = p.x0;
uLowConversion = p.u0;

cases(1).label = 'B';
cases(1).difficulty = 'Baseline';
cases(1).alpha = 0.25;

cases(2).label = 'I';
cases(2).difficulty = 'Intermediate';
cases(2).alpha = 0.50;

cases(3).label = 'C';
cases(3).difficulty = 'Challenge';
cases(3).alpha = 1.00;

for k = 1:numel(cases)
    a = cases(k).alpha;

    cases(k).x0 = ...
        p.xref + a*(xLowConversion-p.xref);

    cases(k).u0 = ...
        p.Tcstar + a*(uLowConversion-p.Tcstar);
end

fprintf('\n============================================================\n');
fprintf('Difficulty instances\n');
fprintf('============================================================\n');

for k = 1:numel(cases)
    fprintf('%s: alpha = %.2f, x0 = [%.6f, %.6f], Tc0 = %.6f K\n', ...
        cases(k).difficulty, ...
        cases(k).alpha, ...
        cases(k).x0(1), ...
        cases(k).x0(2), ...
        cases(k).u0);
end

%% ========================================================================
% 6. Summary storage
% =========================================================================

nCases = numel(cases);

Difficulty = strings(nCases,1);
Alpha = nan(nCases,1);

Initial_CA = nan(nCases,1);
Initial_T_K = nan(nCases,1);
Initial_Tc_K = nan(nCases,1);

Reference_Success = false(nCases,1);
Regulation_Success = false(nCases,1);
Constraints_Satisfied = false(nCases,1);
All_Solver_Steps_Successful = false(nCases,1);

Settling_Time_h = nan(nCases,1);
Normalized_RMSE = nan(nCases,1);
CA_RMSE = nan(nCases,1);
T_RMSE_K = nan(nCases,1);
Terminal_Scaled_Error = nan(nCases,1);
Maximum_Temperature_Deviation_K = nan(nCases,1);

Control_Effort = nan(nCases,1);
Peak_Input_Deviation_K = nan(nCases,1);

Max_State_Violation = nan(nCases,1);
Max_Input_Violation_K = nan(nCases,1);
Max_Input_Rate_Violation_K = nan(nCases,1);

Minimum_CA_Constraint_Margin = nan(nCases,1);
Minimum_T_Constraint_Margin_K = nan(nCases,1);
Minimum_Tc_Constraint_Margin_K = nan(nCases,1);
Minimum_TcRate_Constraint_Margin_K = nan(nCases,1);

Closed_Loop_Cost = nan(nCases,1);

Mean_Solve_Time_s = nan(nCases,1);
Max_Solve_Time_s = nan(nCases,1);
Mean_Iterations = nan(nCases,1);
Solver_Success_Rate = nan(nCases,1);
Total_Closed_Loop_Computation_Time_s = nan(nCases,1);

RK4_vs_ode113_Max_Scaled_Node_Error = nan(nCases,1);
RK4_vs_ode113_Final_Scaled_Error = nan(nCases,1);

%% ========================================================================
% 7. Run Baseline / Intermediate / Challenge
% =========================================================================

for kCase = 1:nCases

    label = cases(kCase).label;
    difficulty = cases(kCase).difficulty;
    alphaCase = cases(kCase).alpha;
    x0 = cases(kCase).x0;
    u0 = cases(kCase).u0;

    Difficulty(kCase) = string(difficulty);
    Alpha(kCase) = alphaCase;
    Initial_CA(kCase) = x0(1);
    Initial_T_K(kCase) = x0(2);
    Initial_Tc_K(kCase) = u0;

    if ~any(runLabels == string(label))
        fprintf('\nSkipping %s because it is not in runLabels.\n', ...
            difficulty);
        continue;
    end

    fprintf('\n============================================================\n');
    fprintf('Running CSTR Constrained Regulation: %s\n',difficulty);
    fprintf('alpha = %.2f\n',alphaCase);
    fprintf('Initial C_A = %.10f kmol/m^3\n',x0(1));
    fprintf('Initial T   = %.10f K\n',x0(2));
    fprintf('Initial Tc  = %.10f K\n',u0);
    fprintf('============================================================\n');

    % Validate the fixed NMPC functions at this instance.
    validateFcns(nlobj,x0,u0);

    %% --------------------------------------------------------------------
    % 7.1 Closed-loop simulation
    % ---------------------------------------------------------------------

    N = round(p.simulationHorizon/p.Ts);

    t = (0:N)*p.Ts;
    tu = t(1:end-1);

    x = zeros(2,N+1);
    u = zeros(1,N);

    solveTime = zeros(1,N);
    exitFlag = zeros(1,N);
    iterations = zeros(1,N);
    predictedCost = nan(1,N);

    x(:,1) = x0;
    lastMV = u0;

    opt = nlmpcmoveopt;

    closedLoopTimer = tic;

    for k = 1:N

        stepTimer = tic;

        [mv,opt,info] = nlmpcmove( ...
            nlobj, ...
            x(:,k), ...
            lastMV, ...
            cfgBase.yref, ...
            [], ...
            opt);

        solveTime(k) = toc(stepTimer);

        % Preserve the behaviour of the existing CSTR-1 implementation:
        % use the manipulated-variable value returned by nlmpcmove.
        u(k) = mv(1);

        exitFlag(k) = info.ExitFlag;
        iterations(k) = info.Iterations;

        if info.ExitFlag >= 0
            predictedCost(k) = info.Cost;
        end

        if info.ExitFlag < 0
            warning('CSTR-1:SolverFailure', ...
                ['NMPC solver returned ExitFlag=%d at t=%.3f h ', ...
                 'for the %s reference.'], ...
                info.ExitFlag,t(k),difficulty);
        end

        x(:,k+1) = cstr_state_dt(x(:,k),mv);

        if any(~isfinite(x(:,k+1)))
            error('Non-finite state encountered in %s at t=%.3f h.', ...
                difficulty,t(k+1));
        end

        lastMV = mv;
    end

    totalClosedLoopTime = toc(closedLoopTimer);

    %% --------------------------------------------------------------------
    % 7.2 Build a result structure compatible with cstr_metrics.m
    %
    % IMPORTANT:
    % cstr_metrics() evaluates the initial coolant move using p.u0.
    % Because Baseline and Intermediate use new benchmark-specific initial
    % coolant values, use a local parameter structure with the correct
    % case-specific u0 before calling cstr_metrics().
    % ---------------------------------------------------------------------

    pCase = p;
    pCase.u0 = u0;

    cfgCase = cfgBase;
    cfgCase.p = pCase;
    cfgCase.x0 = x0;
    cfgCase.u0 = u0;
    cfgCase.Tsim = p.simulationHorizon;

    result.id = cfgCase.id;
    result.t = t;
    result.tu = tu;
    result.x = x;
    result.u = u;
    result.solveTime = solveTime;
    result.exitFlag = exitFlag;
    result.iterations = iterations;
    result.predictedCost = predictedCost;
    result.cfg = cfgCase;

    metrics = cstr_metrics(result);

    %% --------------------------------------------------------------------
    % 7.3 Additional benchmark success checks and constraint margins
    % ---------------------------------------------------------------------

    regulationSuccess = isfinite(metrics.SettlingTime);

    stateConstraintSatisfied = ...
        metrics.MaxStateViolation <= 1e-10;

    inputConstraintSatisfied = ...
        metrics.MaxInputViolation <= 1e-10;

    rateConstraintSatisfied = ...
        metrics.MaxInputRateViolation <= 1e-10;

    constraintsSatisfied = ...
        stateConstraintSatisfied && ...
        inputConstraintSatisfied && ...
        rateConstraintSatisfied;

    allSolverStepsSuccessful = all(exitFlag >= 0);

    referenceSuccess = ...
        regulationSuccess && ...
        constraintsSatisfied && ...
        allSolverStepsSuccessful;

    % Constraint margins. Positive values mean strictly inside the limits.
    caLowerMargin = min(x(1,:) - p.CAMin);
    caUpperMargin = min(p.CAMax - x(1,:));
    minCAMargin = min(caLowerMargin,caUpperMargin);

    tLowerMargin = min(x(2,:) - p.TMin);
    tUpperMargin = min(p.TMax - x(2,:));
    minTMargin = min(tLowerMargin,tUpperMargin);

    tcLowerMargin = min(u - p.TcMin);
    tcUpperMargin = min(p.TcMax - u);
    minTcMargin = min(tcLowerMargin,tcUpperMargin);

    deltaTc = diff([u0,u]);
    tcRateLowerMargin = min(deltaTc - p.TcRateMin);
    tcRateUpperMargin = min(p.TcRateMax - deltaTc);
    minTcRateMargin = min(tcRateLowerMargin,tcRateUpperMargin);

    %% --------------------------------------------------------------------
    % 7.4 Independent ode113 replay of the applied coolant sequence
    % ---------------------------------------------------------------------

    maxScaledNodeError = NaN;
    finalScaledReplayError = NaN;
    xOde113Replay = [];

    if runOde113Replay

        replayOptions = odeset( ...
            'RelTol',1e-11, ...
            'AbsTol',1e-13, ...
            'MaxStep',0.005);

        xOde113Replay = zeros(size(x));
        xOde113Replay(:,1) = x0;

        for k = 1:N

            uk = u(k);

            odeFun = @(tt,xx) cstr_state_ct(xx,uk); %#ok<NASGU>

            [~,xSegment] = ode113( ...
                @(tt,xx)cstr_state_ct(xx,uk), ...
                [0 p.Ts], ...
                xOde113Replay(:,k), ...
                replayOptions);

            xOde113Replay(:,k+1) = xSegment(end,:)';
        end

        scaledReplayError = ...
            (x - xOde113Replay) ./ p.evalScale;

        scaledReplayErrorNorm = ...
            sqrt(sum(scaledReplayError.^2,1));

        maxScaledNodeError = ...
            max(scaledReplayErrorNorm);

        finalScaledReplayError = ...
            scaledReplayErrorNorm(end);
    end

    %% --------------------------------------------------------------------
    % 7.5 Display reference result
    % ---------------------------------------------------------------------

    fprintf('\nREFERENCE RESULT: %s\n',difficulty);
    fprintf('Reference success: %d\n',referenceSuccess);
    fprintf('Regulation success: %d\n',regulationSuccess);
    fprintf('Constraints satisfied: %d\n',constraintsSatisfied);
    fprintf('All NMPC solver steps successful: %d\n', ...
        allSolverStepsSuccessful);

    fprintf('\nPerformance metrics:\n');
    fprintf('Settling time: %.6f h\n',metrics.SettlingTime);
    fprintf('Normalized RMSE: %.9f\n',metrics.NormalizedRMSE);
    fprintf('C_A RMSE: %.9f kmol/m^3\n',metrics.CA_RMSE);
    fprintf('T RMSE: %.9f K\n',metrics.T_RMSE);
    fprintf('Terminal scaled error: %.9e\n', ...
        metrics.TerminalScaledError);
    fprintf('Maximum temperature deviation: %.9f K\n', ...
        metrics.MaxTemperatureDeviation);
    fprintf('Control effort: %.9f\n',metrics.ControlEffort);
    fprintf('Peak input deviation from Tc*: %.9f K\n', ...
        metrics.PeakInputDeviation);
    fprintf('Closed-loop cost: %.9f\n',metrics.ClosedLoopCost);

    fprintf('\nConstraint checks:\n');
    fprintf('Maximum state violation: %.9e\n', ...
        metrics.MaxStateViolation);
    fprintf('Maximum input violation: %.9e K\n', ...
        metrics.MaxInputViolation);
    fprintf('Maximum input-rate violation: %.9e K\n', ...
        metrics.MaxInputRateViolation);
    fprintf('Minimum C_A constraint margin: %.9f kmol/m^3\n', ...
        minCAMargin);
    fprintf('Minimum T constraint margin: %.9f K\n', ...
        minTMargin);
    fprintf('Minimum Tc constraint margin: %.9f K\n', ...
        minTcMargin);
    fprintf('Minimum Delta Tc constraint margin: %.9f K\n', ...
        minTcRateMargin);

    fprintf('\nNMPC solver statistics:\n');
    fprintf('Solver success rate: %.6f\n',metrics.SolverSuccessRate);
    fprintf('Mean solve time: %.6f s\n',metrics.MeanSolveTime);
    fprintf('Maximum solve time: %.6f s\n',metrics.MaxSolveTime);
    fprintf('Mean iterations: %.6f\n',metrics.MeanIterations);
    fprintf('Total closed-loop computation time: %.6f s\n', ...
        totalClosedLoopTime);

    if runOde113Replay
        fprintf('\nRK4-vs-ode113 replay consistency:\n');
        fprintf('Maximum scaled node error: %.9e\n', ...
            maxScaledNodeError);
        fprintf('Final scaled state error: %.9e\n', ...
            finalScaledReplayError);
    end

    %% --------------------------------------------------------------------
    % 7.6 Metadata
    % ---------------------------------------------------------------------

    metadata.system = 'CSTR';
    metadata.id = 'CSTR-1';
    metadata.task = 'Control Design';
    metadata.benchmarkFamily = 'Constrained Regulation';
    metadata.benchmarkRole = 'Structural Challenge';
    metadata.referenceType = 'Analytical + Numerical';

    metadata.difficulty = difficulty;
    metadata.alpha = alphaCase;

    metadata.stateOrder = 'C_A T';
    metadata.input = 'T_c';

    metadata.initialState = x0;
    metadata.initialCoolantTemperature_K = u0;

    metadata.targetState = p.xref;
    metadata.targetCoolantTemperature_K = p.Tcstar;
    metadata.targetResidualInf = targetResidualInf;

    metadata.targetCATolerance = targetCATolerance;
    metadata.targetTTolerance_K = targetTTolerance;

    metadata.CAMin = p.CAMin;
    metadata.CAMax = p.CAMax;
    metadata.TMin_K = p.TMin;
    metadata.TMax_K = p.TMax;
    metadata.TcMin_K = p.TcMin;
    metadata.TcMax_K = p.TcMax;
    metadata.TcRateMin_K_per_interval = p.TcRateMin;
    metadata.TcRateMax_K_per_interval = p.TcRateMax;

    metadata.Ts_h = p.Ts;
    metadata.integrationMethod = ...
        'RK4 with fixed substeps through cstr_state_dt';
    metadata.integrationSubsteps = p.integrationSubsteps;
    metadata.predictionHorizon = p.predictionHorizon;
    metadata.controlHorizon = p.controlHorizon;
    metadata.simulationHorizon_h = p.simulationHorizon;

    metadata.outputWeights = p.outputWeights;
    metadata.mvWeight = p.mvWeight;
    metadata.mvRateWeight = p.mvRateWeight;

    metadata.outputScale = p.outputScale;
    metadata.inputScale = p.inputScale;
    metadata.evalScale = p.evalScale;
    metadata.settlingTolerance = p.settlingTolerance;

    metadata.referenceSuccess = referenceSuccess;
    metadata.regulationSuccess = regulationSuccess;
    metadata.constraintsSatisfied = constraintsSatisfied;
    metadata.allSolverStepsSuccessful = allSolverStepsSuccessful;

    metadata.metrics = metrics;

    metadata.minimumCAMargin = minCAMargin;
    metadata.minimumTemperatureMargin_K = minTMargin;
    metadata.minimumCoolantMargin_K = minTcMargin;
    metadata.minimumCoolantRateMargin_K = minTcRateMargin;

    metadata.totalClosedLoopComputationTime_s = totalClosedLoopTime;

    metadata.ode113ReplayEnabled = runOde113Replay;
    metadata.rk4_vs_ode113_maxScaledNodeError = maxScaledNodeError;
    metadata.rk4_vs_ode113_finalScaledError = finalScaledReplayError;

    metadata.matlabVersion = version;
    metadata.computer = computer;
    metadata.dateGenerated = char(datetime('now'));

    %% --------------------------------------------------------------------
    % 7.7 Save reference MAT file
    % ---------------------------------------------------------------------

    referenceFile = fullfile( ...
        outputFolder, ...
        sprintf('CSTR_CTRL_%s_reference.mat',label));

    save( ...
        referenceFile, ...
        'result', ...
        'metrics', ...
        'metadata', ...
        'p', ...
        'nlobj', ...
        'xOde113Replay', ...
        '-v7.3');

    fprintf('\nSaved reference file:\n%s\n',referenceFile);

    %% --------------------------------------------------------------------
    % 7.8 Save trajectory CSV
    % ---------------------------------------------------------------------

    TcAppliedAtNodes = [u,NaN];
    DeltaTcAtNodes = [deltaTc,NaN];

    if runOde113Replay
        replayCA = xOde113Replay(1,:);
        replayT = xOde113Replay(2,:);
    else
        replayCA = nan(size(t));
        replayT = nan(size(t));
    end

    trajectoryTable = table( ...
        t(:), ...
        x(1,:)', ...
        x(2,:)', ...
        TcAppliedAtNodes(:), ...
        DeltaTcAtNodes(:), ...
        replayCA(:), ...
        replayT(:), ...
        'VariableNames',{ ...
            'Time_h', ...
            'CA_kmol_per_m3', ...
            'Temperature_K', ...
            'Applied_Tc_K', ...
            'Delta_Tc_K', ...
            'Ode113_Replay_CA_kmol_per_m3', ...
            'Ode113_Replay_Temperature_K'});

    trajectoryFile = fullfile( ...
        outputFolder, ...
        sprintf('CSTR_CTRL_%s_trajectory.csv',label));

    writetable(trajectoryTable,trajectoryFile);

    %% --------------------------------------------------------------------
    % 7.9 Summary storage
    % ---------------------------------------------------------------------

    Reference_Success(kCase) = referenceSuccess;
    Regulation_Success(kCase) = regulationSuccess;
    Constraints_Satisfied(kCase) = constraintsSatisfied;
    All_Solver_Steps_Successful(kCase) = allSolverStepsSuccessful;

    Settling_Time_h(kCase) = metrics.SettlingTime;
    Normalized_RMSE(kCase) = metrics.NormalizedRMSE;
    CA_RMSE(kCase) = metrics.CA_RMSE;
    T_RMSE_K(kCase) = metrics.T_RMSE;
    Terminal_Scaled_Error(kCase) = metrics.TerminalScaledError;
    Maximum_Temperature_Deviation_K(kCase) = ...
        metrics.MaxTemperatureDeviation;

    Control_Effort(kCase) = metrics.ControlEffort;
    Peak_Input_Deviation_K(kCase) = metrics.PeakInputDeviation;

    Max_State_Violation(kCase) = metrics.MaxStateViolation;
    Max_Input_Violation_K(kCase) = metrics.MaxInputViolation;
    Max_Input_Rate_Violation_K(kCase) = ...
        metrics.MaxInputRateViolation;

    Minimum_CA_Constraint_Margin(kCase) = minCAMargin;
    Minimum_T_Constraint_Margin_K(kCase) = minTMargin;
    Minimum_Tc_Constraint_Margin_K(kCase) = minTcMargin;
    Minimum_TcRate_Constraint_Margin_K(kCase) = minTcRateMargin;

    Closed_Loop_Cost(kCase) = metrics.ClosedLoopCost;

    Mean_Solve_Time_s(kCase) = metrics.MeanSolveTime;
    Max_Solve_Time_s(kCase) = metrics.MaxSolveTime;
    Mean_Iterations(kCase) = metrics.MeanIterations;
    Solver_Success_Rate(kCase) = metrics.SolverSuccessRate;
    Total_Closed_Loop_Computation_Time_s(kCase) = totalClosedLoopTime;

    RK4_vs_ode113_Max_Scaled_Node_Error(kCase) = maxScaledNodeError;
    RK4_vs_ode113_Final_Scaled_Error(kCase) = finalScaledReplayError;
end

%% ========================================================================
% 8. Final summary
% =========================================================================

summaryTable = table( ...
    Difficulty, ...
    Alpha, ...
    Initial_CA, ...
    Initial_T_K, ...
    Initial_Tc_K, ...
    Reference_Success, ...
    Regulation_Success, ...
    Constraints_Satisfied, ...
    All_Solver_Steps_Successful, ...
    Settling_Time_h, ...
    Normalized_RMSE, ...
    CA_RMSE, ...
    T_RMSE_K, ...
    Terminal_Scaled_Error, ...
    Maximum_Temperature_Deviation_K, ...
    Control_Effort, ...
    Peak_Input_Deviation_K, ...
    Max_State_Violation, ...
    Max_Input_Violation_K, ...
    Max_Input_Rate_Violation_K, ...
    Minimum_CA_Constraint_Margin, ...
    Minimum_T_Constraint_Margin_K, ...
    Minimum_Tc_Constraint_Margin_K, ...
    Minimum_TcRate_Constraint_Margin_K, ...
    Closed_Loop_Cost, ...
    Mean_Solve_Time_s, ...
    Max_Solve_Time_s, ...
    Mean_Iterations, ...
    Solver_Success_Rate, ...
    Total_Closed_Loop_Computation_Time_s, ...
    RK4_vs_ode113_Max_Scaled_Node_Error, ...
    RK4_vs_ode113_Final_Scaled_Error);

fprintf('\n============================================================\n');
fprintf('REFERENCE SUMMARY\n');
fprintf('============================================================\n');

disp(summaryTable);

writetable( ...
    summaryTable, ...
    fullfile(outputFolder,'CSTR_CTRL_reference_summary.csv'));

save( ...
    fullfile(outputFolder,'CSTR_CTRL_reference_summary.mat'), ...
    'summaryTable','cases','p','configurationMetadata');

fprintf('\n============================================================\n');
fprintf('All requested CSTR references completed.\n');
fprintf('============================================================\n');

fprintf('\nExpected official reference files:\n');
fprintf('CSTR_CTRL_B_reference.mat\n');
fprintf('CSTR_CTRL_I_reference.mat\n');
fprintf('CSTR_CTRL_C_reference.mat\n');

fprintf('\nTrajectory files:\n');
fprintf('CSTR_CTRL_B_trajectory.csv\n');
fprintf('CSTR_CTRL_I_trajectory.csv\n');
fprintf('CSTR_CTRL_C_trajectory.csv\n');

fprintf('\nSummary:\n');
fprintf('CSTR_CTRL_reference_summary.csv\n');

fprintf('\nOutput folder:\n%s\n',fullfile(pwd,outputFolder));

end
