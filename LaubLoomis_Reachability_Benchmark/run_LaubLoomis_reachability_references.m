function summaryTable = run_LaubLoomis_reachability_references()
%RUN_LAUBLOOMIS_REACHABILITY_REFERENCES
% A.7 Laub-Loomis — High-Dimensional Nonlinear Reachability
%
% Target environment:
%   MATLAB R2026a
%   CORA v2026.1.0
%
% Canonical 7-state ARCH-COMP Laub-Loomis model:
%
%   x1dot = 1.4*x3 - 0.9*x1
%   x2dot = 2.5*x5 - 1.5*x2
%   x3dot = 0.6*x7 - 0.8*x2*x3
%   x4dot = 2.0 - 1.3*x3*x4
%   x5dot = 0.7*x1 - x4*x5
%   x6dot = 0.3*x1 - 3.1*x6
%   x7dot = 1.8*x6 - 1.5*x2*x7
%
% Initial-box centre:
%   xc = [1.20; 1.05; 1.50; 2.40; 1.00; 0.10; 0.45]
%
% Difficulty instances:
%   Baseline      W = 0.01, unsafe x4 >= 4.5
%   Intermediate  W = 0.05, unsafe x4 >= 4.5
%   Challenge     W = 0.10, unsafe x4 >= 5.0
%
% All cases use:
%   X0 = [xc-W, xc+W]
%   t in [0,20] s
%   autonomous dynamics represented in CORA using a dummy exact-zero input
%
% Primary reference:
%   CORA set-based nonlinear reachability using poly-adaptive.
%
% Independent numerical consistency check:
%   100 sampled initial states per instance, propagated with ode113 using
%   tight tolerances.
%
% Generated files:
%   LaubLoomis_Reachability_References/
%       LAUBLOOMIS_REACH_B_reference.mat
%       LAUBLOOMIS_REACH_I_reference.mat
%       LAUBLOOMIS_REACH_C_reference.mat
%       LAUBLOOMIS_REACH_B_validation.mat
%       LAUBLOOMIS_REACH_I_validation.mat
%       LAUBLOOMIS_REACH_C_validation.mat
%       LAUBLOOMIS_REACH_reference_summary.csv
%       LAUBLOOMIS_REACH_reference_summary.mat
%
% IMPORTANT:
%   The sampled trajectories are consistency checks only. They do not
%   replace the set-based CORA reference or prove safety.
%
% If a long run is interrupted after one or more completed cases, set
% skipExistingReferences = true below and rerun. Completed reference files
% will be loaded into the summary instead of recomputed.

clc;
close all;

%% ========================================================================
% 0. User controls
% =========================================================================

% Run all official benchmark instances.
% To run only one case, e.g. Challenge, use:
% runLabels = ["C"];
runLabels = ["B","I","C"];

% Change to true only when resuming an interrupted run.
skipExistingReferences = false;

% Keep plotting separate from expensive reachability by default.
% The reference data are sufficient to make figures later.
makeValidationFigure = false;

% Number of independent sampled trajectories per case.
numberValidationTrajectories = 100;

%% ========================================================================
% 1. Environment and CORA checks
% =========================================================================

if isempty(which('nonlinearSys')) || isempty(which('reach'))
    error(['CORA cannot be found on the MATLAB path. Add the CORA root ', ...
           'folder before running this script.']);
end

if isempty(which('polyZonotope')) || isempty(which('interval'))
    error('Required CORA set classes cannot be found on the MATLAB path.');
end

fprintf('MATLAB version: %s\n', version);
fprintf('Computer platform: %s\n', computer);

try
    coraVersion = CORAVERSION;
    fprintf('CORA version: %s\n', coraVersion);
catch
    coraVersion = 'CORA version not read automatically';
    warning('CORA version could not be read automatically.');
end

fprintf('\n============================================================\n');
fprintf('A.7 Laub-Loomis Reachability Benchmark\n');
fprintf('============================================================\n');
fprintf('State dimension: 7\n');
fprintf('Horizon: 20.0 s\n');
fprintf('CORA algorithm: poly-adaptive\n');

%% ========================================================================
% 2. Output folder
% =========================================================================

outputFolder = 'LaubLoomis_Reachability_References';

if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end

%% ========================================================================
% 3. Canonical Laub-Loomis dynamics
%
% A dummy scalar input u is retained only to make symbolic insertion
% unambiguous in CORA. params.U is exactly {0}, so the benchmark remains
% autonomous.
% =========================================================================

f = @(x,u) [ ...
    1.4*x(3) - 0.9*x(1)                         + 0*u(1); ...
    2.5*x(5) - 1.5*x(2); ...
    0.6*x(7) - 0.8*x(2)*x(3); ...
    2.0      - 1.3*x(3)*x(4); ...
    0.7*x(1) - x(4)*x(5); ...
    0.3*x(1) - 3.1*x(6); ...
    1.8*x(6) - 1.5*x(2)*x(7) ...
    ];

% IMPORTANT FOR CORA v2026.1.0:
% Do NOT assign a custom system name here. With a custom name such as
% 'LaubLoomis', nonlinearSys creates derivative handles such as
% @jacobian_LaubLoomis and the adaptive reachability path can then try to
% call those functions before automatically generated derivatives exist.
%
% Using the anonymous-function constructor without a custom name follows
% the standard CORA pattern and allows CORA to generate the required
% Jacobian/Hessian derivative files from f.
sys = nonlinearSys(f,7,1);

fprintf('CORA internal system name: %s\n', sys.name);

%% ========================================================================
% 4. Dynamics sanity checks
% =========================================================================

xTest = [1.20;1.05;1.50;2.40;1.00;0.10;0.45];

dxTest = f(xTest,0);

if ~isequal(size(dxTest),[7 1]) || any(~isfinite(dxTest))
    error('Laub-Loomis dynamics do not return a finite 7x1 vector.');
end

fprintf('\nDynamics check at benchmark centre:\n');
disp(dxTest);

% Symbolic dimension test. This directly catches the kind of symbolic
% vector-concatenation problem encountered in earlier CORA runs.
try
    syms xs [7 1] real
    syms us real

    symbolicTest = f(xs,us);

    if ~isequal(size(symbolicTest),[7 1])
        error('Symbolic Laub-Loomis dynamics do not return a 7x1 vector.');
    end

    fprintf('Symbolic dynamics dimension check passed.\n');

catch ME
    fprintf('\nSymbolic dynamics check failed.\n');
    fprintf('%s\n',ME.message);
    rethrow(ME);
end

%% ========================================================================
% 5. Fixed benchmark specification
% =========================================================================

xc = [ ...
    1.20; ...
    1.05; ...
    1.50; ...
    2.40; ...
    1.00; ...
    0.10; ...
    0.45];

Tfinal = 20.0;

cases(1).label = 'B';
cases(1).difficulty = 'Baseline';
cases(1).W = 0.01;
cases(1).unsafeThreshold_x4 = 4.5;

cases(2).label = 'I';
cases(2).difficulty = 'Intermediate';
cases(2).W = 0.05;
cases(2).unsafeThreshold_x4 = 4.5;

cases(3).label = 'C';
cases(3).difficulty = 'Challenge';
cases(3).W = 0.10;
cases(3).unsafeThreshold_x4 = 5.0;

%% ========================================================================
% 6. Common CORA configuration
% =========================================================================

params.tStart = 0;
params.tFinal = Tfinal;

% Exact zero dummy input.
params.U = zonotope(0,0);

% Adaptive polynomial-zonotope nonlinear reachability.
options.alg = 'poly-adaptive';

%% ========================================================================
% 7. Independent trajectory-validation configuration
% =========================================================================

validationRelTol = 1e-12;
validationAbsTol = 1e-14;
validationMaxStep = 0.01;

% A 0.01 s output grid is sufficiently fine for the consistency data while
% avoiding unnecessarily huge validation files.
validationOutputStep = 0.01;
tEval = (0:validationOutputStep:Tfinal)';

odeOptions = odeset( ...
    'RelTol',validationRelTol, ...
    'AbsTol',validationAbsTol, ...
    'MaxStep',validationMaxStep);

% Reproducible sampling.
baseRandomSeed = 2026;

%% ========================================================================
% 8. Summary storage
% =========================================================================

nCases = numel(cases);

Difficulty = strings(nCases,1);
Initial_HalfWidth_W = nan(nCases,1);
Unsafe_Threshold_x4 = nan(nCases,1);

CORA_Max_Upper_x4 = nan(nCases,1);
CORA_Safety_Margin = nan(nCases,1);
CORA_Safe = false(nCases,1);
Final_x4_Width = nan(nCases,1);

Validation_Max_x4 = nan(nCases,1);
Validation_Safe = false(nCases,1);
Validation_Trajectories = nan(nCases,1);

CORA_Computation_Time_s = nan(nCases,1);

%% ========================================================================
% 9. Run B / I / C
% =========================================================================

for kCase = 1:nCases

    label = cases(kCase).label;
    difficulty = cases(kCase).difficulty;
    W = cases(kCase).W;
    unsafeThreshold = cases(kCase).unsafeThreshold_x4;

    Difficulty(kCase) = string(difficulty);
    Initial_HalfWidth_W(kCase) = W;
    Unsafe_Threshold_x4(kCase) = unsafeThreshold;

    % Allow a single-case run without altering the official cases.
    if ~any(runLabels == string(label))
        fprintf('\nSkipping %s because it is not in runLabels.\n',difficulty);
        continue;
    end

    referenceFile = fullfile( ...
        outputFolder, ...
        sprintf('LAUBLOOMIS_REACH_%s_reference.mat',label));

    validationFile = fullfile( ...
        outputFolder, ...
        sprintf('LAUBLOOMIS_REACH_%s_validation.mat',label));

    %% --------------------------------------------------------------------
    % Resume support
    % ---------------------------------------------------------------------

    if skipExistingReferences && exist(referenceFile,'file')

        fprintf('\n============================================================\n');
        fprintf('Loading existing %s reference\n',difficulty);
        fprintf('============================================================\n');

        existing = load(referenceFile,'metadata');

        if ~isfield(existing,'metadata')
            error('Existing reference file does not contain metadata: %s', ...
                referenceFile);
        end

        md = existing.metadata;

        CORA_Computation_Time_s(kCase) = ...
            localFieldOrNaN(md,'computationTime_seconds');

        CORA_Max_Upper_x4(kCase) = ...
            localFieldOrNaN(md,'maximumReachableUpper_x4');

        CORA_Safety_Margin(kCase) = ...
            localFieldOrNaN(md,'safetyMargin_x4');

        Final_x4_Width(kCase) = ...
            localFieldOrNaN(md,'finalReachableWidth_x4');

        if isfield(md,'setBasedSafetySatisfied')
            CORA_Safe(kCase) = logical(md.setBasedSafetySatisfied);
        end

        if isfield(md,'validationMaximum_x4')
            Validation_Max_x4(kCase) = md.validationMaximum_x4;
        end

        if isfield(md,'validationSafetySatisfied')
            Validation_Safe(kCase) = logical(md.validationSafetySatisfied);
        end

        if isfield(md,'validationTrajectoryCount')
            Validation_Trajectories(kCase) = ...
                md.validationTrajectoryCount;
        end

        fprintf('Existing reference loaded successfully.\n');
        continue;
    end

    fprintf('\n============================================================\n');
    fprintf('Running Laub-Loomis Reachability: %s\n',difficulty);
    fprintf('W = %.3f\n',W);
    fprintf('Unsafe region: x4 >= %.3f\n',unsafeThreshold);
    fprintf('============================================================\n');

    %% --------------------------------------------------------------------
    % 9.1 Initial box and polynomial-zonotope conversion
    % ---------------------------------------------------------------------

    lb = xc - W*ones(7,1);
    ub = xc + W*ones(7,1);

    fprintf('Initial lower bound:\n');
    disp(lb');

    fprintf('Initial upper bound:\n');
    disp(ub');

    X0interval = interval(lb,ub);
    params.R0 = polyZonotope(X0interval);

    %% --------------------------------------------------------------------
    % 9.2 CORA set-based reachability
    % ---------------------------------------------------------------------

    reachTimer = tic;

    try
        R = reach(sys,params,options);
    catch ME
        computationTime = toc(reachTimer);

        fprintf('\nERROR during %s CORA reachability.\n',difficulty);
        fprintf('Elapsed time before failure: %.6f s\n',computationTime);
        fprintf('%s\n',ME.message);

        rethrow(ME);
    end

    computationTime = toc(reachTimer);

    fprintf('\nCORA reachability completed successfully.\n');
    fprintf('Computation time: %.6f s\n',computationTime);

    %% --------------------------------------------------------------------
    % 9.3 Extract rigorous x4 enclosure information from the CORA flowpipe
    %
    % Safety is evaluated from time-interval reachable sets whenever they
    % are available. This checks the continuous flowpipe, not merely sampled
    % time points.
    % ---------------------------------------------------------------------

    [maxReachableUpperX4, ...
     minReachableLowerX4, ...
     finalReachableWidthX4, ...
     numberReachSetsUsed, ...
     boundSource] = ...
        localExtractX4Bounds(R);

    safetyMargin = ...
        unsafeThreshold - maxReachableUpperX4;

    setBasedSafetySatisfied = ...
        maxReachableUpperX4 < unsafeThreshold;

    fprintf('\nSet-based x4 enclosure:\n');
    fprintf('Bound source: %s\n',boundSource);
    fprintf('Reach sets inspected: %d\n',numberReachSetsUsed);
    fprintf('Minimum reachable lower x4: %.12f\n', ...
        minReachableLowerX4);
    fprintf('Maximum reachable upper x4: %.12f\n', ...
        maxReachableUpperX4);
    fprintf('Final reachable x4 width: %.12f\n', ...
        finalReachableWidthX4);
    fprintf('Safety threshold: %.12f\n',unsafeThreshold);
    fprintf('Safety margin: %.12f\n',safetyMargin);
    fprintf('Set-based safety satisfied: %d\n', ...
        setBasedSafetySatisfied);

    %% --------------------------------------------------------------------
    % 9.4 Independent high-accuracy trajectory consistency check
    % ---------------------------------------------------------------------

    fprintf('\nGenerating %d independent validation trajectories...\n', ...
        numberValidationTrajectories);

    rng(baseRandomSeed + kCase - 1,'twister');

    validationInitialStates = zeros(7,numberValidationTrajectories);
    validationTrajectories = cell(numberValidationTrajectories,1);

    validationMaximumX4Each = -inf(numberValidationTrajectories,1);

    odeFun = @(t,x) localLaubLoomisODE(x); %#ok<INUSD>

    for j = 1:numberValidationTrajectories

        x0 = lb + (ub-lb).*rand(7,1);

        validationInitialStates(:,j) = x0;

        [tTrajectory,xTrajectory] = ode113( ...
            odeFun, ...
            tEval, ...
            x0, ...
            odeOptions);

        validationTrajectories{j}.t = tTrajectory;
        validationTrajectories{j}.x = xTrajectory;

        validationMaximumX4Each(j) = ...
            max(xTrajectory(:,4));
    end

    validationMaximumX4 = ...
        max(validationMaximumX4Each);

    validationSafetySatisfied = ...
        validationMaximumX4 < unsafeThreshold;

    fprintf('Validation trajectories completed.\n');
    fprintf('Maximum sampled-trajectory x4: %.12f\n', ...
        validationMaximumX4);
    fprintf('Trajectory safety satisfied: %d\n', ...
        validationSafetySatisfied);

    %% --------------------------------------------------------------------
    % 9.5 Metadata
    % ---------------------------------------------------------------------

    metadata.system = ...
        'Laub-Loomis';

    metadata.task = ...
        'Finite-Horizon Reachability';

    metadata.benchmarkFamily = ...
        'High-Dimensional Nonlinear Reachability';

    metadata.benchmarkRole = ...
        'Stress / Scalability';

    metadata.difficulty = ...
        difficulty;

    metadata.stateOrder = ...
        'x1 x2 x3 x4 x5 x6 x7';

    metadata.dimension = ...
        7;

    metadata.initialCenter = ...
        xc;

    metadata.initialHalfWidth_W = ...
        W;

    metadata.initialLowerBound = ...
        lb;

    metadata.initialUpperBound = ...
        ub;

    metadata.tStart = ...
        0;

    metadata.tFinal = ...
        Tfinal;

    metadata.unsafeCondition = ...
        sprintf('x4 >= %.15g',unsafeThreshold);

    metadata.unsafeThreshold_x4 = ...
        unsafeThreshold;

    metadata.algorithm = ...
        'poly-adaptive';

    metadata.initialSetRepresentation = ...
        'Polynomial zonotope';

    metadata.inputSetRepresentation = ...
        'Exact zero dummy-input zonotope';

    metadata.referenceType = ...
        'Literature-based + Numerical';

    metadata.referenceInterpretation = ...
        ['Canonical literature benchmark specification with a fixed ', ...
         'CORA set-based numerical reference. Independent trajectories ', ...
         'are consistency checks only.'];

    metadata.maximumReachableUpper_x4 = ...
        maxReachableUpperX4;

    metadata.minimumReachableLower_x4 = ...
        minReachableLowerX4;

    metadata.finalReachableWidth_x4 = ...
        finalReachableWidthX4;

    metadata.safetyMargin_x4 = ...
        safetyMargin;

    metadata.setBasedSafetySatisfied = ...
        setBasedSafetySatisfied;

    metadata.numberReachSetsUsedForBounds = ...
        numberReachSetsUsed;

    metadata.reachBoundSource = ...
        boundSource;

    metadata.computationTime_seconds = ...
        computationTime;

    metadata.validationTrajectoryCount = ...
        numberValidationTrajectories;

    metadata.validationSolver = ...
        'MATLAB ode113';

    metadata.validationRelTol = ...
        validationRelTol;

    metadata.validationAbsTol = ...
        validationAbsTol;

    metadata.validationMaxStep_seconds = ...
        validationMaxStep;

    metadata.validationOutputStep_seconds = ...
        validationOutputStep;

    metadata.validationMaximum_x4 = ...
        validationMaximumX4;

    metadata.validationSafetySatisfied = ...
        validationSafetySatisfied;

    metadata.randomSeed = ...
        baseRandomSeed + kCase - 1;

    metadata.matlabVersion = ...
        version;

    metadata.computer = ...
        computer;

    metadata.coraVersion = ...
        coraVersion;

    metadata.dateGenerated = ...
        char(datetime('now'));

    %% --------------------------------------------------------------------
    % 9.6 Save primary set-based reference immediately
    % ---------------------------------------------------------------------

    save( ...
        referenceFile, ...
        'R', ...
        'sys', ...
        'params', ...
        'options', ...
        'metadata', ...
        'xc', ...
        'lb', ...
        'ub', ...
        'W', ...
        'unsafeThreshold', ...
        '-v7.3');

    fprintf('\nSaved reference file:\n%s\n',referenceFile);

    %% --------------------------------------------------------------------
    % 9.7 Save independent trajectory validation
    % ---------------------------------------------------------------------

    validationMetadata.description = ...
        ['Independent high-accuracy sampled trajectories used only ', ...
         'as a numerical consistency check for the CORA set-based ', ...
         'reference.'];

    validationMetadata.numberTrajectories = ...
        numberValidationTrajectories;

    validationMetadata.solver = ...
        'MATLAB ode113';

    validationMetadata.RelTol = ...
        validationRelTol;

    validationMetadata.AbsTol = ...
        validationAbsTol;

    validationMetadata.MaxStep_seconds = ...
        validationMaxStep;

    validationMetadata.outputStep_seconds = ...
        validationOutputStep;

    validationMetadata.randomSeed = ...
        baseRandomSeed + kCase - 1;

    validationMetadata.maximum_x4 = ...
        validationMaximumX4;

    validationMetadata.unsafeThreshold_x4 = ...
        unsafeThreshold;

    validationMetadata.safetySatisfied = ...
        validationSafetySatisfied;

    save( ...
        validationFile, ...
        'validationTrajectories', ...
        'validationInitialStates', ...
        'validationMaximumX4Each', ...
        'validationMetadata', ...
        'tEval', ...
        '-v7.3');

    fprintf('Saved validation file:\n%s\n',validationFile);

    %% --------------------------------------------------------------------
    % 9.8 Optional trajectory figure
    %
    % This is intentionally a trajectory-consistency figure rather than a
    % substitute for the CORA flowpipe. A separate plotting script can later
    % render the saved reachSet object R if required for the dissertation.
    % ---------------------------------------------------------------------

    if makeValidationFigure

        figure;
        hold on;
        box on;
        grid on;

        for j = 1:numberValidationTrajectories
            plot( ...
                validationTrajectories{j}.t, ...
                validationTrajectories{j}.x(:,4), ...
                'LineWidth',0.35);
        end

        yline(unsafeThreshold,'--','Unsafe threshold');

        xlabel('Time [s]');
        ylabel('x_4');

        title(sprintf( ...
            'Laub-Loomis sampled validation trajectories - %s', ...
            difficulty));

        exportgraphics( ...
            gcf, ...
            fullfile( ...
                outputFolder, ...
                sprintf( ...
                    'LAUBLOOMIS_REACH_%s_validation_x4.png', ...
                    label)), ...
            'Resolution',300);

        close;
    end

    %% --------------------------------------------------------------------
    % 9.9 Summary
    % ---------------------------------------------------------------------

    CORA_Max_Upper_x4(kCase) = ...
        maxReachableUpperX4;

    CORA_Safety_Margin(kCase) = ...
        safetyMargin;

    CORA_Safe(kCase) = ...
        setBasedSafetySatisfied;

    Final_x4_Width(kCase) = ...
        finalReachableWidthX4;

    Validation_Max_x4(kCase) = ...
        validationMaximumX4;

    Validation_Safe(kCase) = ...
        validationSafetySatisfied;

    Validation_Trajectories(kCase) = ...
        numberValidationTrajectories;

    CORA_Computation_Time_s(kCase) = ...
        computationTime;

    fprintf('\n%s reference completed.\n',difficulty);

end

%% ========================================================================
% 10. Final summary
% =========================================================================

summaryTable = table( ...
    Difficulty, ...
    Initial_HalfWidth_W, ...
    Unsafe_Threshold_x4, ...
    CORA_Max_Upper_x4, ...
    CORA_Safety_Margin, ...
    CORA_Safe, ...
    Final_x4_Width, ...
    Validation_Max_x4, ...
    Validation_Safe, ...
    Validation_Trajectories, ...
    CORA_Computation_Time_s);

fprintf('\n============================================================\n');
fprintf('REFERENCE SUMMARY\n');
fprintf('============================================================\n');

disp(summaryTable);

writetable( ...
    summaryTable, ...
    fullfile( ...
        outputFolder, ...
        'LAUBLOOMIS_REACH_reference_summary.csv'));

save( ...
    fullfile( ...
        outputFolder, ...
        'LAUBLOOMIS_REACH_reference_summary.mat'), ...
    'summaryTable', ...
    'cases', ...
    'xc', ...
    'Tfinal');

fprintf('\n============================================================\n');
fprintf('All requested Laub-Loomis reachability references completed.\n');
fprintf('============================================================\n');

fprintf('\nExpected official reference files:\n');
fprintf('LAUBLOOMIS_REACH_B_reference.mat\n');
fprintf('LAUBLOOMIS_REACH_I_reference.mat\n');
fprintf('LAUBLOOMIS_REACH_C_reference.mat\n');

fprintf('\nOutput folder:\n%s\n',fullfile(pwd,outputFolder));

end


%% =========================================================================
% LOCAL FUNCTION 1
% Pure autonomous Laub-Loomis ODE for ode113 validation
% =========================================================================

function dx = localLaubLoomisODE(x)

dx = [ ...
    1.4*x(3) - 0.9*x(1); ...
    2.5*x(5) - 1.5*x(2); ...
    0.6*x(7) - 0.8*x(2)*x(3); ...
    2.0      - 1.3*x(3)*x(4); ...
    0.7*x(1) - x(4)*x(5); ...
    0.3*x(1) - 3.1*x(6); ...
    1.8*x(6) - 1.5*x(2)*x(7) ...
    ];

end


%% =========================================================================
% LOCAL FUNCTION 2
% Extract x4 bounds from the CORA reachSet object
%
% Preferred source:
%   R.timeInterval.set
%
% because those sets enclose the continuous-time flowpipe between time
% points. If time-interval sets are unavailable, time-point sets are used as
% a compatibility fallback and the source is reported explicitly.
% =========================================================================

function [maxUpperX4, ...
          minLowerX4, ...
          finalWidthX4, ...
          numberSets, ...
          source] = ...
          localExtractX4Bounds(R)

sets = {};
source = '';

% A continuous-system reachSet is normally scalar, but this loop also
% handles reachSet arrays defensively.
for r = 1:numel(R)

    localSets = {};

    % First preference: continuous time-interval sets.
    try
        ti = R(r).timeInterval;

        if isstruct(ti) && isfield(ti,'set') && ~isempty(ti.set)
            localSets = ti.set;
            source = 'R.timeInterval.set';
        elseif isobject(ti) && isprop(ti,'set') && ~isempty(ti.set)
            localSets = ti.set;
            source = 'R.timeInterval.set';
        end
    catch
        % Continue to time-point fallback below.
    end

    % Fallback: time-point sets.
    if isempty(localSets)
        try
            tp = R(r).timePoint;

            if isstruct(tp) && isfield(tp,'set') && ~isempty(tp.set)
                localSets = tp.set;
                source = 'R.timePoint.set (fallback)';
            elseif isobject(tp) && isprop(tp,'set') && ~isempty(tp.set)
                localSets = tp.set;
                source = 'R.timePoint.set (fallback)';
            end
        catch
            % Error after all reachSet elements have been checked.
        end
    end

    if ~isempty(localSets)

        if ~iscell(localSets)
            localSets = num2cell(localSets);
        end

        sets = [sets, localSets]; %#ok<AGROW>
    end
end

if isempty(sets)
    error(['Could not extract reachable sets from the CORA reachSet ', ...
           'object. Inspect properties(R) and update ', ...
           'localExtractX4Bounds if the reachSet storage API has changed.']);
end

numberSets = numel(sets);

lowerX4 = nan(numberSets,1);
upperX4 = nan(numberSets,1);

for i = 1:numberSets

    I = interval(sets{i});

    [lb,ub] = localIntervalBounds(I);

    if numel(lb) < 4 || numel(ub) < 4
        error('A reachable-set interval has dimension smaller than 4.');
    end

    lowerX4(i) = lb(4);
    upperX4(i) = ub(4);
end

if any(~isfinite(lowerX4)) || any(~isfinite(upperX4))
    error('Non-finite x4 bounds were extracted from the CORA flowpipe.');
end

maxUpperX4 = max(upperX4);
minLowerX4 = min(lowerX4);

% The last stored set is used for the final x4 width. For a standard
% continuous reachSet this corresponds to the final flowpipe segment.
finalWidthX4 = upperX4(end) - lowerX4(end);

end


%% =========================================================================
% LOCAL FUNCTION 3
% CORA interval-bound compatibility helper
%
% Current/recent CORA interval objects expose inf/sup. Additional fallbacks
% make the script easier to diagnose if that accessor changes.
% =========================================================================

function [lb,ub] = localIntervalBounds(I)

% Current CORA interface.
try
    lb = I.inf;
    ub = I.sup;

    if isnumeric(lb) && isnumeric(ub)
        lb = lb(:);
        ub = ub(:);
        return;
    end
catch
end

% Possible function-style accessors.
try
    lb = infimum(I);
    ub = supremum(I);

    if isnumeric(lb) && isnumeric(ub)
        lb = lb(:);
        ub = ub(:);
        return;
    end
catch
end

try
    lb = lower(I);
    ub = upper(I);

    if isnumeric(lb) && isnumeric(ub)
        lb = lb(:);
        ub = ub(:);
        return;
    end
catch
end

error(['Unable to read bounds from a CORA interval object. ', ...
       'Run properties(I) in MATLAB and update localIntervalBounds ', ...
       'if the interval accessor API differs in your installation.']);

end


%% =========================================================================
% LOCAL FUNCTION 4
% Metadata helper used by resume mode
% =========================================================================

function value = localFieldOrNaN(S,fieldName)

if isfield(S,fieldName)
    value = S.(fieldName);
else
    value = NaN;
end

end
