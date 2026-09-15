%% Dubins Car Reach-Avoid Numerical Reference Generation
% Final reference-generation script with infeasible straight-line
% start removed and a 120 s wall-clock limit per fmincon start.
%
% Benchmark:
%   A.5.4 Reach-Avoid Steering Control
%
% Model:
%   px_dot  = v*cos(psi)
%   py_dot  = v*sin(psi)
%   psi_dot = u
%
% Fixed speed:
%   v = 1.0 m/s
%
% Steering constraint:
%   |u| <= 1.0 rad/s
%
% Initial state:
%   x0 = [0;0;0]
%
% Target:
%   centre = [5;0]
%   position error <= 0.20 m
%   heading error  <= 10 deg
%
% Circular obstacle:
%   centre = [2.5;0]
%
% Difficulty:
%   Baseline      radius = 0.30 m
%   Intermediate  radius = 0.60 m
%   Challenge     radius = 0.90 m
%
% Maximum arrival horizon:
%   8.0 s
%
% Reference-generation method:
%   - piecewise-constant steering
%   - variable arrival time
%   - multi-start fmincon
%   - exact Dubins propagation inside optimisation
%   - independent high-accuracy ode113 validation
%
% Generates:
%
%   DUBINS_RA_B_reference.mat
%   DUBINS_RA_I_reference.mat
%   DUBINS_RA_C_reference.mat
%
%   DUBINS_RA_B_trajectory.csv
%   DUBINS_RA_I_trajectory.csv
%   DUBINS_RA_C_trajectory.csv
%
%   DUBINS_RA_reference_summary.csv
%
% IMPORTANT:
% The optimisation is used only to generate a fixed numerical
% reference trajectory. Candidate benchmark methods are NOT
% required to use fmincon.


clear;
clc;
close all;


%% ============================================================
% 0. Check Optimization Toolbox
% =============================================================

if isempty(which('fmincon'))

    error([ ...
        'MATLAB fmincon cannot be found. ' ...
        'This reference-generation script requires ' ...
        'Optimization Toolbox.']);

end


fprintf('MATLAB version: %s\n', version);
fprintf('Computer platform: %s\n', computer);


%% ============================================================
% 1. Output folder
% =============================================================

outputFolder = ...
    'Dubins_ReachAvoid_References';

if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end


%% ============================================================
% 2. Fixed Dubins Car benchmark specification
% =============================================================

v = 1.0;                     % m/s
uMax = 1.0;                  % rad/s

x0 = [0;0;0];


% ------------------------------------------------------------
% Target
% ------------------------------------------------------------

targetCentre = [5;0];

targetRadius = 0.20;         % m

% Numerical interior margin used only during reference generation.
% The formal benchmark target remains radius = 0.20 m.
targetPositionBuffer = 0.001; % 1 mm

targetHeading = 0;           % rad

targetHeadingTolerance = ...
    deg2rad(10);             % rad


% ------------------------------------------------------------
% Obstacle
% ------------------------------------------------------------

obstacleCentre = [2.5;0];


% ------------------------------------------------------------
% Benchmark domain
% ------------------------------------------------------------

domain.xMin = -0.5;
domain.xMax =  5.5;

domain.yMin = -2.5;
domain.yMax =  2.5;


% ------------------------------------------------------------
% Maximum benchmark horizon
% ------------------------------------------------------------

Tmax = 8.0;                  % s

% Minimum possible time cannot be less than the straight-line
% distance because v = 1 m/s.
Tmin = 4.8;                  % s


fprintf('\n=============================================\n');
fprintf('Dubins Reach-Avoid benchmark\n');
fprintf('=============================================\n');

fprintf('Speed: %.3f m/s\n', v);
fprintf('Turn-rate bound: +/- %.3f rad/s\n', uMax);

fprintf('Target centre: [%.3f %.3f] m\n', ...
    targetCentre(1), ...
    targetCentre(2));

fprintf('Target radius: %.3f m\n', ...
    targetRadius);

fprintf('Reference-generation target buffer: %.3f m\n', ...
    targetPositionBuffer);

fprintf('Target heading tolerance: %.3f deg\n', ...
    rad2deg(targetHeadingTolerance));

fprintf('Maximum horizon: %.3f s\n', ...
    Tmax);


%% ============================================================
% 3. Difficulty-level cases
% =============================================================

cases(1).label = 'B';
cases(1).difficulty = 'Baseline';
cases(1).obstacleRadius = 0.30;

cases(2).label = 'I';
cases(2).difficulty = 'Intermediate';
cases(2).obstacleRadius = 0.60;

cases(3).label = 'C';
cases(3).difficulty = 'Challenge';
cases(3).obstacleRadius = 0.90;


%% ============================================================
% 4. Control discretisation
%
% u(t) is piecewise constant.
%
% Decision variables:
%
%   z = [u1 u2 ... uN T]'
%
% where T is the terminal / arrival time.
% =============================================================

N = 40;


% Number of exact geometric substeps per control interval used
% while enforcing obstacle and domain constraints.
constraintSubsteps = 20;


% Small numerical clearance used only when generating the
% reference trajectory.
%
% The benchmark obstacle radius itself is NOT changed.
referenceClearanceBuffer = 0.005;    % 5 mm


%% ============================================================
% 5. Optimisation settings
% =============================================================

optimOptions = optimoptions( ...
    'fmincon', ...
    'Algorithm', 'sqp', ...
    'Display', 'none', ...
    'MaxIterations', 1000, ...
    'MaxFunctionEvaluations', 200000, ...
    'ConstraintTolerance', 1e-8, ...
    'OptimalityTolerance', 1e-8, ...
    'StepTolerance', 1e-10);

% Maximum wall-clock time allowed for any single fmincon start.
% A slow or pathological initial guess is stopped, while the best
% feasible solution found from the remaining starts is retained.
maxSecondsPerStart = 120;     % 2 minutes per start


% Decision-variable bounds
lowerBound = [ ...
    -uMax * ones(N,1); ...
    Tmin];


upperBound = [ ...
     uMax * ones(N,1); ...
     Tmax];


%% ============================================================
% 6. Independent validation settings
% =============================================================

validationRelTol = 1e-12;
validationAbsTol = 1e-14;
validationMaxStep = 1e-3;


odeOptions = odeset( ...
    'RelTol', validationRelTol, ...
    'AbsTol', validationAbsTol, ...
    'MaxStep', validationMaxStep);


%% ============================================================
% 7. Summary storage
% =============================================================

summaryDifficulty = strings(3,1);

summaryObstacleRadius = zeros(3,1);

summarySuccess = false(3,1);

summaryArrivalTime = zeros(3,1);

summaryPathLength = zeros(3,1);

summaryMinimumClearance = zeros(3,1);

summaryFinalPositionError = zeros(3,1);

summaryFinalHeadingErrorDeg = zeros(3,1);

summaryPeakTurnRate = zeros(3,1);

summarySteeringEffort = zeros(3,1);

summaryOptimisationTime = zeros(3,1);

summaryValidationDifference = zeros(3,1);

summaryExitFlag = zeros(3,1);


previousBestDecision = [];


%% ============================================================
% 8. Run B / I / C reference optimisations
% =============================================================

for kCase = 1:length(cases)

    label = ...
        cases(kCase).label;

    difficulty = ...
        cases(kCase).difficulty;

    obstacleRadius = ...
        cases(kCase).obstacleRadius;


    fprintf('\n');
    fprintf('=============================================\n');
    fprintf('Running Dubins Reach-Avoid: %s\n', ...
        difficulty);

    fprintf('Obstacle radius = %.3f m\n', ...
        obstacleRadius);

    fprintf('=============================================\n');


    %% --------------------------------------------------------
    % 8.1 Objective
    % ---------------------------------------------------------

    objective = @(z) ...
        localObjective( ...
            z, ...
            N, ...
            x0, ...
            v, ...
            targetCentre, ...
            targetHeading);


    %% --------------------------------------------------------
    % 8.2 Nonlinear constraints
    % ---------------------------------------------------------

    nonlinearConstraints = @(z) ...
        localConstraints( ...
            z, ...
            N, ...
            constraintSubsteps, ...
            x0, ...
            v, ...
            obstacleCentre, ...
            obstacleRadius, ...
            referenceClearanceBuffer, ...
            targetCentre, ...
            targetRadius, ...
            targetPositionBuffer, ...
            targetHeading, ...
            targetHeadingTolerance, ...
            domain);


    %% --------------------------------------------------------
    % 8.3 Construct multi-start initial guesses
    % ---------------------------------------------------------

    initialGuesses = ...
        localGenerateInitialGuesses( ...
            N, ...
            Tmin, ...
            Tmax);


    % Add previous difficulty's best solution as another start.
    if ~isempty(previousBestDecision)

        initialGuesses = [ ...
            previousBestDecision, ...
            initialGuesses];

    end


    numberStarts = ...
        size(initialGuesses,2);


    fprintf('Number of optimisation starts: %d\n', ...
        numberStarts);


    %% --------------------------------------------------------
    % 8.4 Multi-start optimisation
    % ---------------------------------------------------------

    bestFound = false;

    bestObjective = inf;

    bestDecision = [];

    bestExitFlag = NaN;

    bestOutput = [];

    bestConstraintViolation = inf;


    optimisationTimer = tic;


    allStartResults = struct([]);


    for kStart = 1:numberStarts

        z0 = ...
            initialGuesses(:,kStart);


        fprintf('\nStart %d / %d\n', ...
            kStart, ...
            numberStarts);


        try

            % Per-start wall-clock limit prevents one poor initial
            % guess from blocking the complete multi-start run.
            startTimer = tic;

            optimOptionsThisStart = optimoptions( ...
                optimOptions, ...
                'OutputFcn', ...
                @(x,optimValues,state) ...
                    localStopAfterTime( ...
                        x, ...
                        optimValues, ...
                        state, ...
                        startTimer, ...
                        maxSecondsPerStart));

            [zCandidate, ...
             objectiveCandidate, ...
             exitFlagCandidate, ...
             outputCandidate] = ...
                fmincon( ...
                    objective, ...
                    z0, ...
                    [], ...
                    [], ...
                    [], ...
                    [], ...
                    lowerBound, ...
                    upperBound, ...
                    nonlinearConstraints, ...
                    optimOptionsThisStart);


            [cCandidate, ceqCandidate] = ...
                nonlinearConstraints( ...
                    zCandidate);


            maxInequalityViolation = ...
                max([0; cCandidate]);


            if isempty(ceqCandidate)

                maxEqualityViolation = 0;

            else

                maxEqualityViolation = ...
                    max(abs(ceqCandidate));

            end


            maximumViolation = ...
                max( ...
                    maxInequalityViolation, ...
                    maxEqualityViolation);


            fprintf('  objective = %.8f\n', ...
                objectiveCandidate);

            fprintf('  exit flag = %d\n', ...
                exitFlagCandidate);

            fprintf('  terminal time = %.8f s\n', ...
                zCandidate(end));

            fprintf('  max constraint violation = %.3e\n', ...
                maximumViolation);


            allStartResults(kStart).decision = ...
                zCandidate;

            allStartResults(kStart).objective = ...
                objectiveCandidate;

            allStartResults(kStart).exitFlag = ...
                exitFlagCandidate;

            allStartResults(kStart).output = ...
                outputCandidate;

            allStartResults(kStart).maximumConstraintViolation = ...
                maximumViolation;


            candidateFeasible = ...
                isfinite(objectiveCandidate) && ...
                maximumViolation <= 1e-6;


            if candidateFeasible && ...
                    objectiveCandidate < bestObjective

                bestFound = true;

                bestObjective = ...
                    objectiveCandidate;

                bestDecision = ...
                    zCandidate;

                bestExitFlag = ...
                    exitFlagCandidate;

                bestOutput = ...
                    outputCandidate;

                bestConstraintViolation = ...
                    maximumViolation;

            end


        catch ME

            fprintf('  optimisation start failed:\n');
            fprintf('  %s\n', ME.message);

            allStartResults(kStart).decision = [];
            allStartResults(kStart).objective = inf;
            allStartResults(kStart).exitFlag = NaN;
            allStartResults(kStart).output = [];
            allStartResults(kStart).maximumConstraintViolation = inf;

        end

    end


    optimisationTime = ...
        toc(optimisationTimer);


    %% --------------------------------------------------------
    % 8.5 Require a feasible solution
    % ---------------------------------------------------------

    if ~bestFound

        error([ ...
            'No feasible reference trajectory was found for ' ...
            difficulty ...
            '. Do not change the benchmark specification. ' ...
            'Inspect the optimisation output instead.']);

    end


    previousBestDecision = ...
        bestDecision;


    bestControl = ...
        bestDecision(1:N);

    bestTerminalTime = ...
        bestDecision(end);


    fprintf('\nBest feasible solution:\n');

    fprintf('Objective: %.10f\n', ...
        bestObjective);

    fprintf('Terminal time: %.10f s\n', ...
        bestTerminalTime);

    fprintf('Exit flag: %d\n', ...
        bestExitFlag);

    fprintf('Constraint violation: %.3e\n', ...
        bestConstraintViolation);

    fprintf('Total multi-start optimisation time: %.6f s\n', ...
        optimisationTime);


    %% ========================================================
    % 9. Exact reference trajectory from the final control
    % =========================================================

    exactSubsteps = 20;


    [tExact, XExact] = ...
        localPropagateExact( ...
            x0, ...
            bestControl, ...
            bestTerminalTime, ...
            exactSubsteps, ...
            v);


    %% ========================================================
    % 10. Independent ode113 validation
    % =========================================================

    numberValidationPoints = ...
        max( ...
            2001, ...
            ceil(bestTerminalTime / 0.001) + 1);


    tValidation = ...
        linspace( ...
            0, ...
            bestTerminalTime, ...
            numberValidationPoints)';


    dubinsODE = @(t,x) [ ...
        v*cos(x(3)); ...
        v*sin(x(3)); ...
        localPiecewiseControl( ...
            t, ...
            bestControl, ...
            bestTerminalTime) ...
        ];


    [tValidation, XValidation] = ...
        ode113( ...
            dubinsODE, ...
            tValidation, ...
            x0, ...
            odeOptions);


    %% --------------------------------------------------------
    % 10.1 Exact-vs-ode113 endpoint comparison
    % ---------------------------------------------------------

    exactEndpoint = ...
        XExact(end,:)';

    numericalEndpoint = ...
        XValidation(end,:)';


    endpointValidationDifference = ...
        norm( ...
            exactEndpoint - ...
            numericalEndpoint, ...
            inf);


    fprintf('\nIndependent validation:\n');

    fprintf('Exact-vs-ode113 endpoint difference: %.6e\n', ...
        endpointValidationDifference);


    %% ========================================================
    % 11. Reconstruct dense steering history
    % =========================================================

    numberValidationSamples = ...
        length(tValidation);


    uValidation = ...
        zeros(numberValidationSamples,1);


    for j = 1:numberValidationSamples

        uValidation(j) = ...
            localPiecewiseControl( ...
                tValidation(j), ...
                bestControl, ...
                bestTerminalTime);

    end


    %% ========================================================
    % 12. Evaluate benchmark success
    % =========================================================

    px = XValidation(:,1);
    py = XValidation(:,2);
    psi = XValidation(:,3);


    targetDistance = ...
        sqrt( ...
            (px-targetCentre(1)).^2 + ...
            (py-targetCentre(2)).^2);


    headingError = ...
        abs( ...
            localWrapToPi( ...
                psi-targetHeading));


    insideTarget = ...
        targetDistance <= targetRadius & ...
        headingError <= targetHeadingTolerance;


    firstArrivalIndex = ...
        find( ...
            insideTarget, ...
            1, ...
            'first');


    if isempty(firstArrivalIndex)

        reachSuccess = false;

        arrivalTime = NaN;

    else

        reachSuccess = true;

        arrivalTime = ...
            tValidation(firstArrivalIndex);

    end


    %% --------------------------------------------------------
    % 12.1 Obstacle clearance
    % ---------------------------------------------------------

    obstacleDistance = ...
        sqrt( ...
            (px-obstacleCentre(1)).^2 + ...
            (py-obstacleCentre(2)).^2);


    clearance = ...
        obstacleDistance - ...
        obstacleRadius;


    minimumClearance = ...
        min(clearance);


    collisionFree = ...
        minimumClearance >= -1e-8;


    %% --------------------------------------------------------
    % 12.2 Domain constraints
    % ---------------------------------------------------------

    domainSatisfied = ...
        all( ...
            px >= domain.xMin - 1e-9 & ...
            px <= domain.xMax + 1e-9 & ...
            py >= domain.yMin - 1e-9 & ...
            py <= domain.yMax + 1e-9);


    %% --------------------------------------------------------
    % 12.3 Input constraint
    % ---------------------------------------------------------

    peakTurnRate = ...
        max(abs(uValidation));


    inputSatisfied = ...
        peakTurnRate <= uMax + 1e-10;


    %% --------------------------------------------------------
    % 12.4 Overall success
    % ---------------------------------------------------------
    referenceSuccess = ...
        reachSuccess && ...
        collisionFree && ...
        domainSatisfied && ...
        inputSatisfied && ...
        bestTerminalTime <= Tmax + 1e-9;


    fprintf('\nIndependent benchmark validation:\n');

    fprintf('Reach success:       %d\n', ...
        reachSuccess);

    fprintf('Collision free:      %d\n', ...
        collisionFree);

    fprintf('Domain satisfied:    %d\n', ...
        domainSatisfied);

    fprintf('Input satisfied:     %d\n', ...
        inputSatisfied);

    fprintf('Minimum clearance:   %.9f m\n', ...
        minimumClearance);

    fprintf('Terminal distance:   %.9f m\n', ...
        targetDistance(end));

    fprintf('Terminal heading:    %.9f deg\n', ...
        rad2deg(headingError(end)));

    fprintf('Peak turn rate:       %.9f rad/s\n', ...
        peakTurnRate);

    fprintf('Terminal time:        %.9f s\n', ...
        bestTerminalTime);


    if ~referenceSuccess

        error([ ...
            'The optimisation result failed the independent ' ...
            'high-accuracy benchmark validation for ' ...
            difficulty ...
            '.']);

    end
    


    %% ========================================================
    % 13. Reference metrics
    % =========================================================

    if reachSuccess

        metricIndex = ...
            firstArrivalIndex;

    else

        metricIndex = ...
            numberValidationSamples;

    end


    % Constant forward speed means travelled path length is v*t.
    pathLengthToArrival = ...
        v * arrivalTime;


    minimumClearanceToArrival = ...
        min(clearance(1:metricIndex));


    peakTurnRateToArrival = ...
        max(abs(uValidation(1:metricIndex)));


    if metricIndex >= 2

        steeringEffort = ...
            trapz( ...
                tValidation(1:metricIndex), ...
                uValidation(1:metricIndex).^2);

    else

        steeringEffort = 0;

    end


    finalPositionError = ...
        targetDistance(end);


    finalHeadingError = ...
        headingError(end);


    finalHeadingErrorDeg = ...
        rad2deg(finalHeadingError);


    finalState = ...
        XValidation(end,:)';


    fprintf('\n=============================================\n');
    fprintf('REFERENCE RESULT: %s\n', difficulty);
    fprintf('=============================================\n');

    fprintf('Reference success: %d\n', ...
        referenceSuccess);

    fprintf('Target arrival time: %.6f s\n', ...
        arrivalTime);

    fprintf('Path length to arrival: %.6f m\n', ...
        pathLengthToArrival);

    fprintf('Minimum obstacle clearance: %.6f m\n', ...
        minimumClearanceToArrival);

    fprintf('Peak |u| to arrival: %.6f rad/s\n', ...
        peakTurnRateToArrival);

    fprintf('Steering effort: %.6f\n', ...
        steeringEffort);

    fprintf('Final position error: %.6f m\n', ...
        finalPositionError);

    fprintf('Final heading error: %.6f deg\n', ...
        finalHeadingErrorDeg);

    fprintf('Optimised terminal time: %.6f s\n', ...
        bestTerminalTime);

    fprintf('Optimisation time: %.6f s\n', ...
        optimisationTime);


    %% ========================================================
    % 14. Reproducibility metadata
    % =========================================================

    metadata.system = ...
        'Dubins Car';

    metadata.task = ...
        'Control Design';

    metadata.benchmarkFamily = ...
        'Reach-Avoid Steering Control';

    metadata.difficulty = ...
        difficulty;

    metadata.referenceType = ...
        'Numerical';

    metadata.referenceGenerationMethod = ...
        ['Multi-start constrained optimisation using ' ...
         'MATLAB fmincon with piecewise-constant steering'];

    metadata.optimisationAlgorithm = ...
        'SQP';

    metadata.forwardSpeed_mps = ...
        v;

    metadata.turnRateBound_radps = ...
        uMax;

    metadata.initialState = ...
        x0;

    metadata.targetCentre = ...
        targetCentre;

    metadata.targetRadius_m = ...
        targetRadius;

    metadata.targetPositionBuffer_m = ...
        targetPositionBuffer;

    metadata.targetHeading_rad = ...
        targetHeading;

    metadata.targetHeadingTolerance_deg = ...
        rad2deg(targetHeadingTolerance);

    metadata.obstacleCentre = ...
        obstacleCentre;

    metadata.obstacleRadius_m = ...
        obstacleRadius;

    metadata.referenceClearanceBuffer_m = ...
        referenceClearanceBuffer;

    metadata.maximumBenchmarkHorizon_s = ...
        Tmax;

    metadata.numberControlIntervals = ...
        N;

    metadata.optimisedTerminalTime_s = ...
        bestTerminalTime;

    metadata.optimisedControlSequence = ...
        bestControl;

    metadata.optimisationObjective = ...
        bestObjective;

    metadata.optimisationExitFlag = ...
        bestExitFlag;

    metadata.optimisationOutput = ...
        bestOutput;

    metadata.maximumConstraintViolation = ...
        bestConstraintViolation;

    metadata.numberOptimisationStarts = ...
        numberStarts;

    metadata.maximumSecondsPerStart = ...
        maxSecondsPerStart;

    metadata.optimisationTime_seconds = ...
        optimisationTime;

    metadata.validationSolver = ...
        'MATLAB ode113';

    metadata.validationRelTol = ...
        validationRelTol;

    metadata.validationAbsTol = ...
        validationAbsTol;

    metadata.validationMaxStep_seconds = ...
        validationMaxStep;

    metadata.endpointExactVsOde113Error = ...
        endpointValidationDifference;

    metadata.referenceSuccess = ...
        referenceSuccess;

    metadata.targetArrivalTime_s = ...
        arrivalTime;

    metadata.pathLengthToArrival_m = ...
        pathLengthToArrival;

    metadata.minimumObstacleClearance_m = ...
        minimumClearanceToArrival;

    metadata.peakTurnRate_radps = ...
        peakTurnRateToArrival;

    metadata.steeringEffort = ...
        steeringEffort;

    metadata.finalPositionError_m = ...
        finalPositionError;

    metadata.finalHeadingError_deg = ...
        finalHeadingErrorDeg;

    metadata.matlabVersion = ...
        version;

    metadata.computer = ...
        computer;

    metadata.dateGenerated = ...
        char(datetime('now'));


    %% ========================================================
    % 15. Save complete reference MAT file
    % =========================================================

    referenceFile = fullfile( ...
        outputFolder, ...
        sprintf( ...
            'DUBINS_RA_%s_reference.mat', ...
            label));


    save( ...
        referenceFile, ...
        'bestControl', ...
        'bestTerminalTime', ...
        'bestObjective', ...
        'bestExitFlag', ...
        'allStartResults', ...
        'tExact', ...
        'XExact', ...
        'tValidation', ...
        'XValidation', ...
        'uValidation', ...
        'metadata', ...
        'v', ...
        'uMax', ...
        'x0', ...
        'targetCentre', ...
        'targetRadius', ...
        'targetPositionBuffer', ...
        'targetHeading', ...
        'targetHeadingTolerance', ...
        'obstacleCentre', ...
        'obstacleRadius', ...
        'domain', ...
        '-v7.3');


    fprintf('\nSaved reference:\n');
    fprintf('%s\n', referenceFile);


    %% ========================================================
    % 16. Save trajectory CSV
    % =========================================================

    trajectoryTable = table( ...
        tValidation, ...
        XValidation(:,1), ...
        XValidation(:,2), ...
        XValidation(:,3), ...
        uValidation, ...
        targetDistance, ...
        headingError, ...
        clearance, ...
        insideTarget, ...
        'VariableNames', { ...
            'time_s', ...
            'px_m', ...
            'py_m', ...
            'psi_rad', ...
            'turn_rate_radps', ...
            'target_distance_m', ...
            'heading_error_rad', ...
            'obstacle_clearance_m', ...
            'inside_target'});


    trajectoryCSV = fullfile( ...
        outputFolder, ...
        sprintf( ...
            'DUBINS_RA_%s_trajectory.csv', ...
            label));


    writetable( ...
        trajectoryTable, ...
        trajectoryCSV);


    %% ========================================================
    % 17. Plot reach-avoid trajectory
    % =========================================================

    figure;

    hold on;
    box on;
    grid on;


    % Reference trajectory
    plot( ...
        px, ...
        py, ...
        'LineWidth', ...
        1.5);


    % Initial state
    plot( ...
        x0(1), ...
        x0(2), ...
        'o', ...
        'MarkerSize', ...
        8, ...
        'LineWidth', ...
        1.5);


    % Obstacle
    angleGrid = ...
        linspace(0,2*pi,400);

    obstacleX = ...
        obstacleCentre(1) + ...
        obstacleRadius*cos(angleGrid);

    obstacleY = ...
        obstacleCentre(2) + ...
        obstacleRadius*sin(angleGrid);


    plot( ...
        obstacleX, ...
        obstacleY, ...
        'LineWidth', ...
        1.5);


    % Target
    targetX = ...
        targetCentre(1) + ...
        targetRadius*cos(angleGrid);

    targetY = ...
        targetCentre(2) + ...
        targetRadius*sin(angleGrid);


    plot( ...
        targetX, ...
        targetY, ...
        '--', ...
        'LineWidth', ...
        1.5);


    xlabel('p_x [m]');
    ylabel('p_y [m]');

    title(sprintf( ...
        'Dubins Reach-Avoid Reference - %s', ...
        difficulty));


    axis equal;

    xlim([domain.xMin domain.xMax]);
    ylim([domain.yMin domain.yMax]);


    trajectoryFigure = fullfile( ...
        outputFolder, ...
        sprintf( ...
            'DUBINS_RA_%s_trajectory.png', ...
            label));


    exportgraphics( ...
        gcf, ...
        trajectoryFigure, ...
        'Resolution', ...
        300);


    savefig( ...
        fullfile( ...
            outputFolder, ...
            sprintf( ...
                'DUBINS_RA_%s_trajectory.fig', ...
                label)));


    close;


    %% ========================================================
    % 18. Plot steering input
    % =========================================================

    figure;

    plot( ...
        tValidation, ...
        uValidation, ...
        'LineWidth', ...
        1.2);

    hold on;

    yline(uMax,'--');
    yline(-uMax,'--');

    grid on;
    box on;

    xlabel('Time [s]');
    ylabel('Turn rate u [rad/s]');

    title(sprintf( ...
        'Dubins Reach-Avoid Steering - %s', ...
        difficulty));


    steeringFigure = fullfile( ...
        outputFolder, ...
        sprintf( ...
            'DUBINS_RA_%s_steering.png', ...
            label));


    exportgraphics( ...
        gcf, ...
        steeringFigure, ...
        'Resolution', ...
        300);

    close;


    %% ========================================================
    % 19. Store summary
    % =========================================================

    summaryDifficulty(kCase) = ...
        string(difficulty);

    summaryObstacleRadius(kCase) = ...
        obstacleRadius;

    summarySuccess(kCase) = ...
        referenceSuccess;

    summaryArrivalTime(kCase) = ...
        arrivalTime;

    summaryPathLength(kCase) = ...
        pathLengthToArrival;

    summaryMinimumClearance(kCase) = ...
        minimumClearanceToArrival;

    summaryFinalPositionError(kCase) = ...
        finalPositionError;

    summaryFinalHeadingErrorDeg(kCase) = ...
        finalHeadingErrorDeg;

    summaryPeakTurnRate(kCase) = ...
        peakTurnRateToArrival;

    summarySteeringEffort(kCase) = ...
        steeringEffort;

    summaryOptimisationTime(kCase) = ...
        optimisationTime;

    summaryValidationDifference(kCase) = ...
        endpointValidationDifference;

    summaryExitFlag(kCase) = ...
        bestExitFlag;

end


%% ============================================================
% 20. Reference summary table
% =============================================================

summaryTable = table( ...
    summaryDifficulty, ...
    summaryObstacleRadius, ...
    summarySuccess, ...
    summaryArrivalTime, ...
    summaryPathLength, ...
    summaryMinimumClearance, ...
    summaryFinalPositionError, ...
    summaryFinalHeadingErrorDeg, ...
    summaryPeakTurnRate, ...
    summarySteeringEffort, ...
    summaryOptimisationTime, ...
    summaryValidationDifference, ...
    summaryExitFlag, ...
    'VariableNames', { ...
        'Difficulty', ...
        'Obstacle_Radius_m', ...
        'Reference_Success', ...
        'Arrival_Time_s', ...
        'Path_Length_m', ...
        'Minimum_Clearance_m', ...
        'Final_Position_Error_m', ...
        'Final_Heading_Error_deg', ...
        'Peak_Turn_Rate_radps', ...
        'Steering_Effort', ...
        'Optimisation_Time_s', ...
        'Exact_vs_ode113_Error', ...
        'Exit_Flag'});


fprintf('\n');
fprintf('=============================================\n');
fprintf('REFERENCE SUMMARY\n');
fprintf('=============================================\n');

disp(summaryTable);


summaryCSV = fullfile( ...
    outputFolder, ...
    'DUBINS_RA_reference_summary.csv');


writetable( ...
    summaryTable, ...
    summaryCSV);


save( ...
    fullfile( ...
        outputFolder, ...
        'DUBINS_RA_reference_summary.mat'), ...
    'summaryTable');


%% ============================================================
% 21. Finished
% =============================================================

fprintf('\n');
fprintf('=============================================\n');
fprintf('All Dubins Reach-Avoid references completed.\n');
fprintf('=============================================\n');

fprintf('\nGenerated reference files:\n');

fprintf('DUBINS_RA_B_reference.mat\n');
fprintf('DUBINS_RA_I_reference.mat\n');
fprintf('DUBINS_RA_C_reference.mat\n');

fprintf('\nTrajectory files:\n');

fprintf('DUBINS_RA_B_trajectory.csv\n');
fprintf('DUBINS_RA_I_trajectory.csv\n');
fprintf('DUBINS_RA_C_trajectory.csv\n');

fprintf('\nSummary:\n');

fprintf('DUBINS_RA_reference_summary.csv\n');

fprintf('\nOutput folder:\n');

disp(fullfile(pwd, outputFolder));


%% ============================================================
% LOCAL FUNCTION 1
% Objective function
% =============================================================

function J = localObjective( ...
    z, ...
    N, ...
    x0, ...
    v, ...
    targetCentre, ...
    targetHeading)


    u = z(1:N);

    T = z(end);


    % Exact trajectory at control-interval endpoints
    [~,X] = ...
        localPropagateExact( ...
            x0, ...
            u, ...
            T, ...
            1, ...
            v);


    xFinal = X(end,:)';


    terminalPositionError = ...
        norm( ...
            xFinal(1:2) - ...
            targetCentre);


    terminalHeadingError = ...
        localWrapToPi( ...
            xFinal(3) - ...
            targetHeading);


    dt = T / N;


    steeringEffort = ...
        dt * sum(u.^2);


    % Time is the dominant objective.
    %
    % Small regularisation terms encourage:
    % - target-centre accuracy
    % - final heading accuracy
    % - moderate steering effort

    J = ...
        T ...
        + 0.02 * steeringEffort ...
        + 2.0 * terminalPositionError^2 ...
        + 0.5 * terminalHeadingError^2;

end


%% ============================================================
% LOCAL FUNCTION 2
% Nonlinear benchmark constraints
% =============================================================

function [c,ceq] = localConstraints( ...
    z, ...
    N, ...
    substeps, ...
    x0, ...
    v, ...
    obstacleCentre, ...
    obstacleRadius, ...
    clearanceBuffer, ...
    targetCentre, ...
    targetRadius, ...
    targetPositionBuffer, ...
    targetHeading, ...
    targetHeadingTolerance, ...
    domain)


    u = z(1:N);

    T = z(end);


    [~,X] = ...
        localPropagateExact( ...
            x0, ...
            u, ...
            T, ...
            substeps, ...
            v);


    px = X(:,1);
    py = X(:,2);


    %% --------------------------------------------------------
    % Obstacle avoidance
    %
    % fmincon requires c <= 0.
    %
    % (r+buffer)^2 - distance^2 <= 0
    % ---------------------------------------------------------

    obstacleDistanceSquared = ...
        (px-obstacleCentre(1)).^2 + ...
        (py-obstacleCentre(2)).^2;


    cObstacle = ...
        (obstacleRadius + clearanceBuffer)^2 ...
        - obstacleDistanceSquared;


    %% --------------------------------------------------------
    % Domain constraints
    % ---------------------------------------------------------

    cDomain = [ ...
        domain.xMin - px; ...
        px - domain.xMax; ...
        domain.yMin - py; ...
        py - domain.yMax];


    %% --------------------------------------------------------
    % Terminal target constraints
    % ---------------------------------------------------------

    xFinal = X(end,:)';


    terminalDistance = ...
        norm( ...
            xFinal(1:2) - ...
            targetCentre);


    terminalHeadingError = ...
        abs( ...
            localWrapToPi( ...
                xFinal(3) - ...
                targetHeading));


    cTargetPosition = ...
        terminalDistance - ...
        (targetRadius - targetPositionBuffer);


    cTargetHeading = ...
        terminalHeadingError - ...
        targetHeadingTolerance;


    %% --------------------------------------------------------
    % Complete inequality vector
    % ---------------------------------------------------------

    c = [ ...
        cObstacle; ...
        cDomain; ...
        cTargetPosition; ...
        cTargetHeading];


    ceq = [];

end


%% ============================================================
% LOCAL FUNCTION 3
% Generate multi-start initial control guesses
% =============================================================

function guesses = localGenerateInitialGuesses( ...
    N, ...
    Tmin, ...
    Tmax)


    guesses = [];


    %% --------------------------------------------------------
    % Straight-line initial guess intentionally omitted.
    %
    % The obstacle is centred on the direct path from (0,0) to
    % (5,0), so zero steering is infeasible for every difficulty
    % level and can lead to unnecessarily slow SQP iterations.
    % ---------------------------------------------------------


    %% --------------------------------------------------------
    % Symmetric S-turn guesses
    %
    % Pattern:
    %
    % +a for 1/4
    % -a for 1/2
    % +a for 1/4
    %
    % This produces near-zero final heading and returns towards
    % the target centre while passing above the obstacle.
    %
    % Negative mirror passes below the obstacle.
    % ---------------------------------------------------------

    amplitudes = [ ...
        0.4, ...
        0.7, ...
        0.9];


    times = [ ...
        5.5, ...
        6.0, ...
        6.5];


    nFirst = floor(N/4);

    nMiddle = floor(N/2);

    nLast = ...
        N - nFirst - nMiddle;


    for k = 1:length(amplitudes)

        a = amplitudes(k);

        Tguess = min( ...
            max(times(k),Tmin), ...
            Tmax);


        uPositive = [ ...
            a*ones(nFirst,1); ...
           -a*ones(nMiddle,1); ...
            a*ones(nLast,1)];


        uNegative = ...
            -uPositive;


        guesses(:,end+1) = [ ...
            uPositive; ...
            Tguess];


        guesses(:,end+1) = [ ...
            uNegative; ...
            Tguess];

    end

end


%% ============================================================
% LOCAL FUNCTION 4
% Exact piecewise-constant Dubins propagation
% =============================================================

function [tGrid,X] = localPropagateExact( ...
    x0, ...
    uSequence, ...
    T, ...
    substeps, ...
    v)


    N = length(uSequence);


    dtControl = ...
        T / N;


    dt = ...
        dtControl / substeps;


    totalSteps = ...
        N * substeps;


    X = zeros( ...
        totalSteps + 1, ...
        3);


    tGrid = zeros( ...
        totalSteps + 1, ...
        1);


    X(1,:) = ...
        x0';


    x = x0;


    counter = 1;


    for k = 1:N

        uk = ...
            uSequence(k);


        for j = 1:substeps

            x = ...
                localExactDubinsStep( ...
                    x, ...
                    uk, ...
                    dt, ...
                    v);


            counter = ...
                counter + 1;


            X(counter,:) = ...
                x';


            tGrid(counter) = ...
                tGrid(counter-1) + ...
                dt;

        end

    end


    % Avoid tiny floating-point drift in the final time.
    tGrid(end) = T;

end


%% ============================================================
% LOCAL FUNCTION 5
% Exact constant-control Dubins step
% =============================================================

function xNext = localExactDubinsStep( ...
    x, ...
    u, ...
    dt, ...
    v)


    px = x(1);
    py = x(2);
    psi = x(3);


    if abs(u) < 1e-12

        pxNext = ...
            px + ...
            v*dt*cos(psi);


        pyNext = ...
            py + ...
            v*dt*sin(psi);


        psiNext = ...
            psi;


    else

        psiNext = ...
            psi + ...
            u*dt;


        pxNext = ...
            px + ...
            (v/u) * ...
            ( ...
            sin(psiNext) - ...
            sin(psi) ...
            );


        pyNext = ...
            py - ...
            (v/u) * ...
            ( ...
            cos(psiNext) - ...
            cos(psi) ...
            );

    end


    xNext = [ ...
        pxNext; ...
        pyNext; ...
        psiNext];

end


%% ============================================================
% LOCAL FUNCTION 6
% Piecewise-constant control lookup
% =============================================================

function u = localPiecewiseControl( ...
    t, ...
    controlSequence, ...
    T)


    N = length(controlSequence);


    dt = T / N;


    index = ...
        floor(t / dt) + 1;


    index = ...
        max( ...
            1, ...
            min(index,N));


    u = ...
        controlSequence(index);

end


%% ============================================================
% LOCAL FUNCTION 7
% Per-start wall-clock time limit for fmincon
% =============================================================

function stop = localStopAfterTime( ...
    ~, ...
    ~, ...
    state, ...
    startTimer, ...
    maxSeconds)

    stop = false;

    if strcmp(state,'iter') && ...
            toc(startTimer) >= maxSeconds

        stop = true;

    end

end


%% ============================================================
% LOCAL FUNCTION 8
% Wrap angle into [-pi,pi]
% =============================================================

function angleWrapped = localWrapToPi(angle)


    angleWrapped = ...
        mod( ...
            angle + pi, ...
            2*pi) ...
        - pi;

end