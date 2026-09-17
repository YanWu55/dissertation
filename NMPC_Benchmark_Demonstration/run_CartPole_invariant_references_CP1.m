function summaryTable = run_CartPole_invariant_references_CP1()
%RUN_CARTPOLE_INVARIANT_REFERENCES_CP1
% A.6.4 Controlled Lyapunov Invariant-Set Certification
%
% Shared plant: the ACTUAL CP-1 model from cartpole_params.m /
% cartpole_state_ct.m.
%
% State order:
%   x = [z; theta; z_dot; theta_dot]
%
% Plant parameters, track bounds and force bounds are loaded directly from
% cartpole_params.m.
%
% Fixed reconstructed LQR — exactly the same specification used by the
% revised A.6.5 reference generator:
%
%   Q = diag([10 150 1 5])
%   R = 0.2
%
% The continuous-time plant is numerically linearised at the upright
% equilibrium. Candidate Lyapunov sets:
%
%   Omega_rho = {x : x' P x <= rho}
%
%   Baseline      rho = 0.25
%   Intermediate  rho = 0.60
%   Challenge     rho = 1.00
%
% For each rho this program:
%   1. computes exact ellipsoidal bounds on |z|, |theta|, and |-Kx|;
%   2. evaluates 30,000 nonlinear boundary points;
%   3. refines high-Vdot points using multi-start fmincon;
%   4. validates 105 nonlinear boundary trajectories for 5 s;
%   5. stores a high-accuracy numerical certification reference.
%
% IMPORTANT:
% This is numerical certification, not a formal interval-arithmetic proof.

clc;
close all;

%% ------------------------------------------------------------------------
% 0. Requirements
% -------------------------------------------------------------------------

if isempty(which('cartpole_params')) || isempty(which('cartpole_state_ct'))
    error(['Place this file in the same project folder as ', ...
        'cartpole_params.m and cartpole_state_ct.m, or add that folder ', ...
        'to the MATLAB path.']);
end

if isempty(which('lqr'))
    error('lqr() was not found. Control System Toolbox is required.');
end

if isempty(which('fmincon'))
    error('fmincon() was not found. Optimization Toolbox is required.');
end

fprintf('MATLAB version: %s\n',version);
fprintf('Computer platform: %s\n',computer);

%% ------------------------------------------------------------------------
% 1. Load the actual shared CP-1 plant
% -------------------------------------------------------------------------

p = cartpole_params();

uAbsLimit = min(abs([p.uMin,p.uMax]));
zAbsLimit = min(abs([p.zMin,p.zMax]));

fprintf('\n============================================================\n');
fprintf('A.6.4 Cart-Pole Invariant Sets — shared CP-1 plant\n');
fprintf('============================================================\n');
fprintf('State order: [z theta z_dot theta_dot]\n');
fprintf('M  = %.6f kg\n',p.M);
fprintf('m  = %.6f kg\n',p.m);
fprintf('l  = %.6f m\n',p.l);
fprintf('g  = %.6f m/s^2\n',p.g);
fprintf('Kd = %.6f N s/m\n',p.Kd);
fprintf('Track constraint: [%.3f, %.3f] m\n',p.zMin,p.zMax);
fprintf('Force constraint: [%.3f, %.3f] N\n',p.uMin,p.uMax);

if abs(p.uMin + p.uMax) > 1e-12
    warning(['The current certification code interprets the force limit ', ...
        'through the smaller absolute bound because the CP-1 bounds are ', ...
        'not exactly symmetric.']);
end

if abs(p.zMin + p.zMax) > 1e-12
    warning(['The current certification code interprets the track limit ', ...
        'through the smaller absolute bound because the CP-1 bounds are ', ...
        'not exactly symmetric.']);
end

outputFolder = 'CartPole_Invariant_References';
if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end

%% ------------------------------------------------------------------------
% 2. Exact shared-plant dynamics consistency check
% -------------------------------------------------------------------------

xChecks = [ ...
     0.00, deg2rad( 0),  0.00,  0.00; ...
     0.12, deg2rad( 4),  0.08, -0.12; ...
    -0.20, deg2rad(-7), -0.15,  0.20; ...
     0.35, deg2rad(10),  0.25, -0.30]';

uChecks = [0, 20, -35, 50];

for j = 1:size(xChecks,2)
    d1 = localPlantDynamics(xChecks(:,j),uChecks(j),p);
    d2 = cartpole_state_ct(xChecks(:,j),uChecks(j));

    if norm(d1-d2,inf) > 1e-11
        error('Local invariant-set dynamics mismatch cartpole_state_ct at check %d.',j);
    end
end

fprintf('CP-1 dynamics consistency checks passed.\n');

%% ------------------------------------------------------------------------
% 3. Numerical linearisation of the ACTUAL nonlinear CP-1 plant
% -------------------------------------------------------------------------

xEq = p.xref(:);
uEq = p.u0;

[A,B] = localNumericalLinearization(xEq,uEq,p);

controllabilityRank = rank(ctrb(A,B));

Q = diag([10,150,1,5]);
R = 0.2;

[K,P,closedLoopEigenvalues] = lqr(A,B,Q,R);

P = 0.5*(P+P');

Acl = A-B*K;
isHurwitz = all(real(closedLoopEigenvalues) < 0);

careResidual = A'*P + P*A - P*B*(R\(B'*P)) + Q;
careResidualNorm = norm(careResidual,'fro');

fprintf('\n============================================================\n');
fprintf('Fixed reconstructed LQR shared with A.6.5\n');
fprintf('============================================================\n');
fprintf('Controllability rank: %d / 4\n',controllabilityRank);
fprintf('Q = diag([10 150 1 5])\n');
fprintf('R = 0.2\n');
fprintf('\nA =\n');
disp(A);
fprintf('B =\n');
disp(B);
fprintf('K =\n');
disp(K);
fprintf('P =\n');
disp(P);
fprintf('Closed-loop eigenvalues:\n');
disp(closedLoopEigenvalues);
fprintf('A-BK Hurwitz: %d\n',isHurwitz);
fprintf('CARE residual Frobenius norm: %.6e\n',careResidualNorm);

if controllabilityRank < 4
    error('The actual CP-1 upright linearisation is not controllable.');
end

if ~isHurwitz
    error('The reconstructed CP-1 LQR closed loop is not Hurwitz.');
end

pEigenvalues = eig(P);
if any(pEigenvalues <= 0)
    error('The LQR Riccati matrix P is not positive definite.');
end

fprintf('Minimum eigenvalue of P: %.6e\n',min(pEigenvalues));

%% ------------------------------------------------------------------------
% 4. Candidate set levels
% -------------------------------------------------------------------------

cases(1).label = 'B';
cases(1).difficulty = 'Baseline';
cases(1).rho = 0.25;

cases(2).label = 'I';
cases(2).difficulty = 'Intermediate';
cases(2).rho = 0.60;

cases(3).label = 'C';
cases(3).difficulty = 'Challenge';
cases(3).rho = 1.00;

%% ------------------------------------------------------------------------
% 5. Boundary search configuration
% -------------------------------------------------------------------------

numberBoundarySamples = 30000;
numberOptimisationStarts = 20;

rng(2026,'twister');

vdotTolerance = 1e-8;

optimOptions = optimoptions( ...
    'fmincon', ...
    'Algorithm','sqp', ...
    'Display','none', ...
    'MaxIterations',500, ...
    'MaxFunctionEvaluations',50000, ...
    'ConstraintTolerance',1e-10, ...
    'OptimalityTolerance',1e-10, ...
    'StepTolerance',1e-12);

%% ------------------------------------------------------------------------
% 6. Independent nonlinear trajectory validation
% -------------------------------------------------------------------------

numberRandomValidationTrajectories = 100;
numberCriticalTrajectories = 5;
numberValidationTrajectories = ...
    numberRandomValidationTrajectories + numberCriticalTrajectories;

validationHorizon = 5.0;
validationOutputStep = 0.005;

tEval = (0:validationOutputStep:validationHorizon)';

validationRelTol = 1e-10;
validationAbsTol = 1e-12;
validationMaxStep = 0.005;

odeOptions = odeset( ...
    'RelTol',validationRelTol, ...
    'AbsTol',validationAbsTol, ...
    'MaxStep',validationMaxStep);

trajectoryVTolerance = 1e-7;

%% ------------------------------------------------------------------------
% 7. Ellipsoid transformations
% -------------------------------------------------------------------------

Pinv = inv(P);

[VP,DP] = eig(P);
PinvHalf = VP * diag(1./sqrt(diag(DP))) * VP';

%% ------------------------------------------------------------------------
% 8. Save common controller reference
% -------------------------------------------------------------------------

controllerMetadata.system = 'Cart-Pole';
controllerMetadata.task = 'Controlled Invariant-Set Analysis';
controllerMetadata.benchmarkFamily = ...
    'Controlled Lyapunov Invariant-Set Certification';
controllerMetadata.sharedPlant = 'CP-1 actual benchmark configuration';
controllerMetadata.stateOrder = 'z theta z_dot theta_dot';
controllerMetadata.angleConvention = 'theta = 0 is upright';

controllerMetadata.cartMass_kg = p.M;
controllerMetadata.poleMass_kg = p.m;
controllerMetadata.poleLength_m = p.l;
controllerMetadata.gravity_mps2 = p.g;
controllerMetadata.cartDamping_Ns_per_m = p.Kd;
controllerMetadata.cartPositionMin_m = p.zMin;
controllerMetadata.cartPositionMax_m = p.zMax;
controllerMetadata.forceMin_N = p.uMin;
controllerMetadata.forceMax_N = p.uMax;

controllerMetadata.lqrStatus = ...
    'Reconstructed fixed LQR shared with revised A.6.5';
controllerMetadata.Q = Q;
controllerMetadata.R = R;
controllerMetadata.K = K;
controllerMetadata.P = P;
controllerMetadata.A = A;
controllerMetadata.B = B;
controllerMetadata.closedLoopEigenvalues = closedLoopEigenvalues;
controllerMetadata.isHurwitz = isHurwitz;
controllerMetadata.controllabilityRank = controllabilityRank;
controllerMetadata.careResidualNorm = careResidualNorm;
controllerMetadata.matlabVersion = version;
controllerMetadata.computer = computer;
controllerMetadata.dateGenerated = char(datetime('now'));

save(fullfile(outputFolder,'CARTPOLE_INV_controller_reference.mat'), ...
    'A','B','Q','R','K','P','Acl','closedLoopEigenvalues', ...
    'controllerMetadata','p');

%% ------------------------------------------------------------------------
% 9. Summary storage
% -------------------------------------------------------------------------

Difficulty = strings(3,1);
rhoColumn = zeros(3,1);
Reference_Certified = false(3,1);
Maximum_Boundary_Vdot = zeros(3,1);
Maximum_Unsaturated_Force_N = zeros(3,1);
Force_Margin_N = zeros(3,1);
Maximum_Position_m = zeros(3,1);
Position_Margin_m = zeros(3,1);
Maximum_Theta_deg = zeros(3,1);
Trajectory_Checks_Passed = false(3,1);
Maximum_Trajectory_V_Excess = zeros(3,1);
Computation_Time_s = zeros(3,1);

%% ------------------------------------------------------------------------
% 10. Run B / I / C
% -------------------------------------------------------------------------

for kCase = 1:numel(cases)

    label = cases(kCase).label;
    difficulty = cases(kCase).difficulty;
    rho = cases(kCase).rho;

    fprintf('\n============================================================\n');
    fprintf('Running Cart-Pole Invariant Set: %s\n',difficulty);
    fprintf('rho = %.6f\n',rho);
    fprintf('============================================================\n');

    caseTimer = tic;

    %% Exact ellipsoidal component/input bounds
    eZ = [1;0;0;0];
    eTheta = [0;1;0;0];

    maxPositionExact = sqrt(rho*(eZ'*Pinv*eZ));
    maxThetaExact = sqrt(rho*(eTheta'*Pinv*eTheta));
    maxThetaDeg = rad2deg(maxThetaExact);

    maxUnsaturatedForceExact = sqrt(rho*(K*Pinv*K'));

    forceMargin = uAbsLimit - maxUnsaturatedForceExact;
    positionMargin = zAbsLimit - maxPositionExact;

    noSaturationThroughoutSet = ...
        maxUnsaturatedForceExact <= uAbsLimit + 1e-10;

    positionConstraintThroughoutSet = ...
        maxPositionExact <= zAbsLimit + 1e-10;

    fprintf('\nExact ellipsoidal bounds:\n');
    fprintf('max |z|       = %.9f m\n',maxPositionExact);
    fprintf('position margin = %.9f m\n',positionMargin);
    fprintf('max |theta|   = %.9f deg\n',maxThetaDeg);
    fprintf('max |-Kx|     = %.9f N\n',maxUnsaturatedForceExact);
    fprintf('force margin  = %.9f N\n',forceMargin);
    fprintf('No saturation anywhere in Omega_rho: %d\n', ...
        noSaturationThroughoutSet);

    %% Dense random boundary sample
    Z = randn(4,numberBoundarySamples);
    Z = Z ./ vecnorm(Z,2,1);

    Xboundary = sqrt(rho)*PinvHalf*Z;

    sampledVdot = zeros(numberBoundarySamples,1);
    sampledAppliedForce = zeros(numberBoundarySamples,1);
    sampledUnsaturatedForce = zeros(numberBoundarySamples,1);

    for j = 1:numberBoundarySamples
        xj = Xboundary(:,j);

        [vdot_j,uApplied_j,uUnsaturated_j] = ...
            localVdot(xj,P,K,p);

        sampledVdot(j) = vdot_j;
        sampledAppliedForce(j) = uApplied_j;
        sampledUnsaturatedForce(j) = uUnsaturated_j;
    end

    [maxSampledVdot,maxSampledIndex] = max(sampledVdot);
    xWorstSample = Xboundary(:,maxSampledIndex);

    fprintf('\nBoundary sampling:\n');
    fprintf('Boundary samples: %d\n',numberBoundarySamples);
    fprintf('Largest sampled Vdot: %.12e\n',maxSampledVdot);

    %% Select top sampled Vdot points for fmincon refinement
    [~,sortIndex] = sort(sampledVdot,'descend');
    optimisationStartIndices = sortIndex( ...
        1:min(numberOptimisationStarts,numberBoundarySamples));

    componentBounds = sqrt(rho*diag(Pinv));
    lbOpt = -componentBounds;
    ubOpt = componentBounds;

    nonlinearBoundaryConstraint = ...
        @(x) localEllipsoidBoundaryConstraint(x,P,rho);

    objective = @(x) -localVdotOnly(x,P,K,p);

    bestOptimisedVdot = -inf;
    xWorstOptimised = xWorstSample;
    optimisationResults = struct([]);

    for kStart = 1:numel(optimisationStartIndices)

        xStart = Xboundary(:,optimisationStartIndices(kStart));

        try
            [xCandidate,objectiveCandidate,exitFlagCandidate,outputCandidate] = ...
                fmincon( ...
                    objective, ...
                    xStart, ...
                    [],[],[],[], ...
                    lbOpt,ubOpt, ...
                    nonlinearBoundaryConstraint, ...
                    optimOptions);

            candidateVdot = -objectiveCandidate;
            boundaryResidual = abs(xCandidate'*P*xCandidate-rho);

            optimisationResults(kStart).x = xCandidate;
            optimisationResults(kStart).Vdot = candidateVdot;
            optimisationResults(kStart).exitFlag = exitFlagCandidate;
            optimisationResults(kStart).output = outputCandidate;
            optimisationResults(kStart).boundaryResidual = boundaryResidual;

            if boundaryResidual <= 1e-7 && candidateVdot > bestOptimisedVdot
                bestOptimisedVdot = candidateVdot;
                xWorstOptimised = xCandidate;
            end

        catch ME
            optimisationResults(kStart).x = [];
            optimisationResults(kStart).Vdot = NaN;
            optimisationResults(kStart).exitFlag = NaN;
            optimisationResults(kStart).output = [];
            optimisationResults(kStart).boundaryResidual = NaN;
            optimisationResults(kStart).errorMessage = ME.message;
        end
    end

    %% Adopt largest identified Vdot
    if isfinite(bestOptimisedVdot) && bestOptimisedVdot > maxSampledVdot
        maxBoundaryVdot = bestOptimisedVdot;
        xWorstVdot = xWorstOptimised;
        maxVdotSource = 'fmincon refinement';
    else
        maxBoundaryVdot = maxSampledVdot;
        xWorstVdot = xWorstSample;
        maxVdotSource = 'dense boundary sampling';
    end

    fprintf('\nBoundary optimisation:\n');
    fprintf('Maximum identified Vdot: %.12e\n',maxBoundaryVdot);
    fprintf('Maximum source: %s\n',maxVdotSource);
    fprintf('Worst boundary state [z theta z_dot theta_dot]:\n');
    disp(xWorstVdot');

    vdotConditionSatisfied = maxBoundaryVdot < -vdotTolerance;

    %% Critical deterministic boundary states
    forceDenominator = sqrt(K*Pinv*K');

    xForcePositive = sqrt(rho)*Pinv*K'/forceDenominator;
    xForceNegative = -xForcePositive;

    positionDenominator = sqrt(eZ'*Pinv*eZ);

    xPositionPositive = sqrt(rho)*Pinv*eZ/positionDenominator;
    xPositionNegative = -xPositionPositive;

    %% Independent random + critical boundary trajectories
    Zvalidation = randn(4,numberRandomValidationTrajectories);
    Zvalidation = Zvalidation ./ vecnorm(Zvalidation,2,1);

    X0Validation = sqrt(rho)*PinvHalf*Zvalidation;

    X0Validation = [ ...
        X0Validation, ...
        xWorstVdot, ...
        xForcePositive, ...
        xForceNegative, ...
        xPositionPositive, ...
        xPositionNegative];

    validationTrajectories = cell(numberValidationTrajectories,1);
    trajectoryPassFlags = false(numberValidationTrajectories,1);
    trajectoryMaxV = zeros(numberValidationTrajectories,1);
    trajectoryMaxPosition = zeros(numberValidationTrajectories,1);
    trajectoryMaxAppliedForce = zeros(numberValidationTrajectories,1);
    trajectoryMaxUnsaturatedForce = zeros(numberValidationTrajectories,1);

    closedLoopODE = @(t,x) localClosedLoopDynamics(x,K,p); %#ok<INUSD>

    fprintf('\nRunning %d nonlinear boundary trajectories...\n', ...
        numberValidationTrajectories);

    for j = 1:numberValidationTrajectories

        x0 = X0Validation(:,j);

        [tTrajectory,xTrajectory] = ode113( ...
            closedLoopODE,tEval,x0,odeOptions);

        numberSamples = size(xTrajectory,1);

        Vtrajectory = zeros(numberSamples,1);
        appliedForceTrajectory = zeros(numberSamples,1);
        unsaturatedForceTrajectory = zeros(numberSamples,1);

        for q = 1:numberSamples
            xq = xTrajectory(q,:)';

            Vtrajectory(q) = xq'*P*xq;

            [uApplied_q,uUnsaturated_q] = localController(xq,K,p);

            appliedForceTrajectory(q) = uApplied_q;
            unsaturatedForceTrajectory(q) = uUnsaturated_q;
        end

        maxV_j = max(Vtrajectory);
        maxPosition_j = max(abs(xTrajectory(:,1)));
        maxAppliedForce_j = max(abs(appliedForceTrajectory));
        maxUnsaturatedForce_j = max(abs(unsaturatedForceTrajectory));

        trajectoryPass_j = ...
            maxV_j <= rho + trajectoryVTolerance && ...
            maxPosition_j <= zAbsLimit + 1e-9 && ...
            maxAppliedForce_j <= uAbsLimit + 1e-9;

        validationTrajectories{j}.t = tTrajectory;
        validationTrajectories{j}.x = xTrajectory;
        validationTrajectories{j}.V = Vtrajectory;
        validationTrajectories{j}.uApplied = appliedForceTrajectory;
        validationTrajectories{j}.uUnsaturated = ...
            unsaturatedForceTrajectory;

        trajectoryPassFlags(j) = trajectoryPass_j;
        trajectoryMaxV(j) = maxV_j;
        trajectoryMaxPosition(j) = maxPosition_j;
        trajectoryMaxAppliedForce(j) = maxAppliedForce_j;
        trajectoryMaxUnsaturatedForce(j) = maxUnsaturatedForce_j;
    end

    allTrajectoryChecksPassed = all(trajectoryPassFlags);

    maxTrajectoryV = max(trajectoryMaxV);
    maxTrajectoryVExcess = maxTrajectoryV-rho;
    maxTrajectoryPosition = max(trajectoryMaxPosition);
    maxTrajectoryAppliedForce = max(trajectoryMaxAppliedForce);
    maxTrajectoryUnsaturatedForce = max(trajectoryMaxUnsaturatedForce);

    fprintf('Trajectory checks passed: %d\n',allTrajectoryChecksPassed);
    fprintf('Maximum trajectory V-rho: %.12e\n',maxTrajectoryVExcess);
    fprintf('Maximum trajectory |z|: %.9f m\n',maxTrajectoryPosition);
    fprintf('Maximum trajectory applied |u|: %.9f N\n', ...
        maxTrajectoryAppliedForce);

    %% Adopted numerical certification
    referenceCertified = ...
        vdotConditionSatisfied && ...
        positionConstraintThroughoutSet && ...
        noSaturationThroughoutSet && ...
        allTrajectoryChecksPassed;

    computationTime = toc(caseTimer);

    fprintf('\n============================================================\n');
    fprintf('REFERENCE RESULT: %s\n',difficulty);
    fprintf('============================================================\n');
    fprintf('rho: %.6f\n',rho);
    fprintf('Maximum identified boundary Vdot: %.12e\n',maxBoundaryVdot);
    fprintf('Vdot condition satisfied: %d\n',vdotConditionSatisfied);
    fprintf('Maximum ellipsoidal |z|: %.9f m\n',maxPositionExact);
    fprintf('Position margin: %.9f m\n',positionMargin);
    fprintf('Maximum ellipsoidal |-Kx|: %.9f N\n', ...
        maxUnsaturatedForceExact);
    fprintf('Force margin: %.9f N\n',forceMargin);
    fprintf('All trajectory checks passed: %d\n', ...
        allTrajectoryChecksPassed);
    fprintf('Reference certified: %d\n',referenceCertified);
    fprintf('Computation time: %.6f s\n',computationTime);

    %% Metadata
    metadata.system = 'Cart-Pole';
    metadata.task = 'Controlled Invariant-Set Analysis';
    metadata.benchmarkFamily = ...
        'Controlled Lyapunov Invariant-Set Certification';
    metadata.sharedPlant = 'CP-1 actual benchmark configuration';
    metadata.difficulty = difficulty;
    metadata.rho = rho;
    metadata.stateOrder = 'z theta z_dot theta_dot';
    metadata.angleConvention = 'theta = 0 is upright';
    metadata.referenceType = 'Analytical + Numerical';
    metadata.referenceInterpretation = [ ...
        'Quadratic Lyapunov structure from the reconstructed CP-1 LQR ', ...
        'with high-accuracy nonlinear boundary search and trajectory ', ...
        'validation. Not a formal interval-arithmetic proof.'];

    metadata.cartMass_kg = p.M;
    metadata.poleMass_kg = p.m;
    metadata.poleLength_m = p.l;
    metadata.gravity_mps2 = p.g;
    metadata.cartDamping_Ns_per_m = p.Kd;

    metadata.forceMin_N = p.uMin;
    metadata.forceMax_N = p.uMax;
    metadata.cartPositionMin_m = p.zMin;
    metadata.cartPositionMax_m = p.zMax;

    metadata.Q = Q;
    metadata.R = R;
    metadata.K = K;
    metadata.P = P;
    metadata.A = A;
    metadata.B = B;
    metadata.closedLoopEigenvalues = closedLoopEigenvalues;
    metadata.careResidualNorm = careResidualNorm;

    metadata.numberBoundarySamples = numberBoundarySamples;
    metadata.numberOptimisationStarts = numberOptimisationStarts;
    metadata.vdotTolerance = vdotTolerance;
    metadata.maximumBoundaryVdot = maxBoundaryVdot;
    metadata.maximumVdotSource = maxVdotSource;
    metadata.worstVdotState = xWorstVdot;

    metadata.maximumPositionExact_m = maxPositionExact;
    metadata.positionMargin_m = positionMargin;
    metadata.maximumThetaExact_deg = maxThetaDeg;
    metadata.maximumUnsaturatedForceExact_N = ...
        maxUnsaturatedForceExact;
    metadata.forceMargin_N = forceMargin;
    metadata.noSaturationThroughoutSet = noSaturationThroughoutSet;
    metadata.positionConstraintThroughoutSet = ...
        positionConstraintThroughoutSet;

    metadata.numberValidationTrajectories = ...
        numberValidationTrajectories;
    metadata.validationHorizon_s = validationHorizon;
    metadata.validationSolver = 'MATLAB ode113';
    metadata.validationRelTol = validationRelTol;
    metadata.validationAbsTol = validationAbsTol;
    metadata.validationMaxStep_s = validationMaxStep;
    metadata.allTrajectoryChecksPassed = allTrajectoryChecksPassed;
    metadata.maximumTrajectoryVExcess = maxTrajectoryVExcess;
    metadata.maximumTrajectoryPosition_m = maxTrajectoryPosition;
    metadata.maximumTrajectoryAppliedForce_N = maxTrajectoryAppliedForce;
    metadata.maximumTrajectoryUnsaturatedForce_N = ...
        maxTrajectoryUnsaturatedForce;

    metadata.referenceCertified = referenceCertified;
    metadata.computationTime_seconds = computationTime;
    metadata.randomSeed = 2026;
    metadata.matlabVersion = version;
    metadata.computer = computer;
    metadata.dateGenerated = char(datetime('now'));

    %% Save reference
    referenceFile = fullfile(outputFolder, ...
        sprintf('CARTPOLE_INV_%s_reference.mat',label));

    save(referenceFile, ...
        'A','B','Q','R','K','P','Acl','rho', ...
        'Xboundary','sampledVdot','xWorstVdot', ...
        'optimisationResults','validationTrajectories', ...
        'X0Validation','metadata','p','-v7.3');

    fprintf('\nSaved reference file:\n%s\n',referenceFile);

    %% Validation summary CSV
    validationNumber = (1:numberValidationTrajectories)';

    validationTable = table( ...
        validationNumber, ...
        trajectoryPassFlags, ...
        trajectoryMaxV, ...
        trajectoryMaxPosition, ...
        trajectoryMaxAppliedForce, ...
        trajectoryMaxUnsaturatedForce, ...
        'VariableNames',{ ...
            'Trajectory', ...
            'Pass', ...
            'Maximum_V', ...
            'Maximum_abs_z_m', ...
            'Maximum_Applied_Force_N', ...
            'Maximum_Unsaturated_Force_N'});

    writetable(validationTable, ...
        fullfile(outputFolder, ...
        sprintf('CARTPOLE_INV_%s_validation.csv',label)));

    %% Figures
    figure;
    scatter( ...
        Xboundary(1,1:10:end), ...
        rad2deg(Xboundary(2,1:10:end)), ...
        5,'filled');
    grid on;
    box on;
    xlabel('Cart position z [m]');
    ylabel('Pole angle \theta [deg]');
    title(sprintf('Cart-Pole Candidate Invariant Set - %s',difficulty));

    exportgraphics(gcf, ...
        fullfile(outputFolder, ...
        sprintf('CARTPOLE_INV_%s_z_theta.png',label)), ...
        'Resolution',300);
    close;

    figure;
    hold on;
    box on;
    grid on;

    for j = 1:numberValidationTrajectories
        plot( ...
            validationTrajectories{j}.t, ...
            validationTrajectories{j}.V./rho, ...
            'LineWidth',0.4);
    end

    yline(1,'--','Boundary');
    xlabel('Time [s]');
    ylabel('V(x(t)) / \rho');
    title(sprintf('Cart-Pole Invariant-Set Validation - %s',difficulty));

    exportgraphics(gcf, ...
        fullfile(outputFolder, ...
        sprintf('CARTPOLE_INV_%s_V_validation.png',label)), ...
        'Resolution',300);
    close;

    %% Summary
    Difficulty(kCase) = string(difficulty);
    rhoColumn(kCase) = rho;
    Reference_Certified(kCase) = referenceCertified;
    Maximum_Boundary_Vdot(kCase) = maxBoundaryVdot;
    Maximum_Unsaturated_Force_N(kCase) = maxUnsaturatedForceExact;
    Force_Margin_N(kCase) = forceMargin;
    Maximum_Position_m(kCase) = maxPositionExact;
    Position_Margin_m(kCase) = positionMargin;
    Maximum_Theta_deg(kCase) = maxThetaDeg;
    Trajectory_Checks_Passed(kCase) = allTrajectoryChecksPassed;
    Maximum_Trajectory_V_Excess(kCase) = maxTrajectoryVExcess;
    Computation_Time_s(kCase) = computationTime;
end

%% ------------------------------------------------------------------------
% 11. Final summary
% -------------------------------------------------------------------------

rho = rhoColumn; %#ok<NASGU>

summaryTable = table( ...
    Difficulty, ...
    rhoColumn, ...
    Reference_Certified, ...
    Maximum_Boundary_Vdot, ...
    Maximum_Unsaturated_Force_N, ...
    Force_Margin_N, ...
    Maximum_Position_m, ...
    Position_Margin_m, ...
    Maximum_Theta_deg, ...
    Trajectory_Checks_Passed, ...
    Maximum_Trajectory_V_Excess, ...
    Computation_Time_s, ...
    'VariableNames',{ ...
        'Difficulty', ...
        'rho', ...
        'Reference_Certified', ...
        'Maximum_Boundary_Vdot', ...
        'Maximum_Unsaturated_Force_N', ...
        'Force_Margin_N', ...
        'Maximum_Position_m', ...
        'Position_Margin_m', ...
        'Maximum_Theta_deg', ...
        'Trajectory_Checks_Passed', ...
        'Maximum_Trajectory_V_Excess', ...
        'Computation_Time_s'});

fprintf('\n============================================================\n');
fprintf('REFERENCE SUMMARY\n');
fprintf('============================================================\n');
disp(summaryTable);

writetable(summaryTable, ...
    fullfile(outputFolder,'CARTPOLE_INV_reference_summary.csv'));

save(fullfile(outputFolder,'CARTPOLE_INV_reference_summary.mat'), ...
    'summaryTable','K','P','A','B','closedLoopEigenvalues','p');

fprintf('\n============================================================\n');
fprintf('All shared-plant CP-1 invariant-set references completed.\n');
fprintf('============================================================\n');
fprintf('Output folder:\n%s\n',fullfile(pwd,outputFolder));

end


%% =========================================================================
% LOCAL FUNCTION: actual CP-1 continuous-time dynamics
% =========================================================================

function dx = localPlantDynamics(x,u,p)

zDot = x(3);
theta = x(2);
thetaDot = x(4);
F = u(1);

s = sin(theta);
c = cos(theta);

den = p.M + p.m*s^2;

zDDot = ( ...
    F ...
    - p.Kd*zDot ...
    - p.m*p.l*thetaDot^2*s ...
    + p.m*p.g*s*c ...
    ) / den;

thetaDDot = ( ...
    p.g*s ...
    + (F - p.Kd*zDot - p.m*p.l*thetaDot^2*s)*c/(p.M+p.m) ...
    ) / ( ...
    p.l - p.m*p.l*c^2/(p.M+p.m) ...
    );

dx = [zDot; thetaDot; zDDot; thetaDDot];

end


%% =========================================================================
% LOCAL FUNCTION: numerical linearisation of the actual nonlinear CP-1 plant
% =========================================================================

function [A,B] = localNumericalLinearization(xEq,uEq,p)

nx = numel(xEq);
A = zeros(nx,nx);
B = zeros(nx,1);

epsX = 1e-6;
epsU = 1e-6;

for i = 1:nx
    dx = zeros(nx,1);
    dx(i) = epsX;

    fp = localPlantDynamics(xEq+dx,uEq,p);
    fm = localPlantDynamics(xEq-dx,uEq,p);

    A(:,i) = (fp-fm)/(2*epsX);
end

fp = localPlantDynamics(xEq,uEq+epsU,p);
fm = localPlantDynamics(xEq,uEq-epsU,p);

B(:,1) = (fp-fm)/(2*epsU);

end


%% =========================================================================
% LOCAL FUNCTION: saturated reconstructed LQR
% =========================================================================

function [uApplied,uUnsaturated] = localController(x,K,p)

uUnsaturated = p.u0 - K*(x-p.xref);

uApplied = min(max(uUnsaturated,p.uMin),p.uMax);

end


%% =========================================================================
% LOCAL FUNCTION: nonlinear closed-loop dynamics
% =========================================================================

function dx = localClosedLoopDynamics(x,K,p)

[uApplied,~] = localController(x,K,p);

dx = localPlantDynamics(x,uApplied,p);

end


%% =========================================================================
% LOCAL FUNCTION: Lyapunov derivative
% =========================================================================

function [Vdot,uApplied,uUnsaturated] = localVdot(x,P,K,p)

[uApplied,uUnsaturated] = localController(x,K,p);

dx = localPlantDynamics(x,uApplied,p);

Vdot = 2*x'*P*dx;

end


%% =========================================================================
% LOCAL FUNCTION: scalar Vdot wrapper
% =========================================================================

function value = localVdotOnly(x,P,K,p)

value = localVdot(x,P,K,p);

end


%% =========================================================================
% LOCAL FUNCTION: ellipsoid boundary equality
% =========================================================================

function [c,ceq] = localEllipsoidBoundaryConstraint(x,P,rho)

c = [];
ceq = x'*P*x-rho;

end
