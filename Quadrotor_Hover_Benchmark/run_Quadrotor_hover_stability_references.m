%% Quadrotor Closed-Loop Hover Stability Reference Generation
%
% MATLAB R2025a
%
% Benchmark:
%   A.4.3 Quadrotor - Closed-Loop Hover Stability
%   and Nonlinear Recovery
%
% State:
%   x = [px py pz vx vy vz phi theta psi p q r]'
%
% Input:
%   u = [T tau_phi tau_theta tau_psi]'
%
% Fixed parameters:
%   m = 1.0 kg
%   g = 9.81 m/s^2
%   J = diag([0.02 0.02 0.04]) kg m^2
%
% Hover target:
%   position = [0 0 1] m
%   velocity = 0
%   Euler angles = 0
%   angular velocity = 0
%
% Fixed controller:
%   Continuous-time LQR constructed from hover linearisation
%
% Generates:
%
%   QUAD_HOVER_B_reference.mat
%   QUAD_HOVER_I_reference.mat
%   QUAD_HOVER_C_reference.mat
%
%   QUAD_HOVER_B_trajectory.csv
%   QUAD_HOVER_I_trajectory.csv
%   QUAD_HOVER_C_trajectory.csv
%
%   QUAD_HOVER_reference_summary.csv
%   QUAD_HOVER_controller_reference.mat
%
% plus figures.


clear;
clc;
close all;


%% ============================================================
% 0. Output folder
% =============================================================

outputFolder = 'Quadrotor_Hover_References';

if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end


fprintf('MATLAB version: %s\n', version);
fprintf('Computer platform: %s\n', computer);


%% ============================================================
% 1. Quadrotor parameters
% =============================================================

quad.m = 1.0;                      % kg
quad.g = 9.81;                     % m/s^2

quad.J = diag([ ...
    0.02, ...
    0.02, ...
    0.04]);                        % kg m^2


% ------------------------------------------------------------
% Actuator limits
% ------------------------------------------------------------

quad.Tmin = 0.0;                   % N
quad.Tmax = 19.62;                 % N

quad.tauMax = [ ...
    0.50; ...
    0.50; ...
    0.20];                         % N m


%% ============================================================
% 2. Hover equilibrium
% =============================================================

% State ordering:
%
%  1 px
%  2 py
%  3 pz
%  4 vx
%  5 vy
%  6 vz
%  7 phi
%  8 theta
%  9 psi
% 10 p
% 11 q
% 12 r

xEq = zeros(12,1);

xEq(1:3) = [ ...
    0; ...
    0; ...
    1];


uEq = [ ...
    quad.m * quad.g; ...
    0; ...
    0; ...
    0];


fprintf('\n=============================================\n');
fprintf('Hover equilibrium\n');
fprintf('=============================================\n');

fprintf('Hover position: [%.3f %.3f %.3f] m\n', ...
    xEq(1), xEq(2), xEq(3));

fprintf('Hover thrust: %.6f N\n', uEq(1));


%% ============================================================
% 3. Linearised hover model
% =============================================================

A = zeros(12,12);
B = zeros(12,4);


% Position kinematics
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;


% Small-angle translation coupling
%
% x_ddot = g*theta
% y_ddot = -g*phi
% z_ddot = DeltaT/m

A(4,8) = quad.g;
A(5,7) = -quad.g;


% Euler-angle kinematics near hover
A(7,10) = 1;
A(8,11) = 1;
A(9,12) = 1;


% Input matrix
B(6,1)  = 1 / quad.m;

B(10,2) = 1 / quad.J(1,1);
B(11,3) = 1 / quad.J(2,2);
B(12,4) = 1 / quad.J(3,3);


%% ============================================================
% 4. LQR weighting matrices
% =============================================================

Q = diag([ ...
    10, ...     % px
    10, ...     % py
    20, ...     % pz
     5, ...     % vx
     5, ...     % vy
    10, ...     % vz
    50, ...     % phi
    50, ...     % theta
    20, ...     % psi
     5, ...     % p
     5, ...     % q
     5]);       % r


R = diag([ ...
    1.0, ...
    0.1, ...
    0.1, ...
    0.1]);


%% ============================================================
% 5. Check controllability
% =============================================================

n = size(A,1);

ControllabilityMatrix = B;

AB = B;

for j = 1:(n-1)

    AB = A * AB;

    ControllabilityMatrix = [ ...
        ControllabilityMatrix, ...
        AB]; %#ok<AGROW>

end


controllabilityRank = rank(ControllabilityMatrix);


fprintf('\n=============================================\n');
fprintf('Linearised system check\n');
fprintf('=============================================\n');

fprintf('State dimension: %d\n', n);
fprintf('Controllability rank: %d\n', ...
    controllabilityRank);


if controllabilityRank < n

    error(['Linearised hover system is not fully ' ...
           'controllable.']);

end


%% ============================================================
% 6. Compute fixed LQR gain
%
% First try MATLAB lqr().
% If Control System Toolbox is unavailable, use a Hamiltonian
% CARE solution implemented at the bottom of this script.
% =============================================================

if ~isempty(which('lqr'))

    fprintf('\nUsing MATLAB lqr().\n');

    [K, P, closedLoopEigenvalues] = ...
        lqr(A, B, Q, R);

    lqrMethod = 'MATLAB lqr';

else

    fprintf('\n');
    fprintf(['MATLAB lqr() not found. ' ...
             'Using Hamiltonian CARE fallback.\n']);

    [K, P] = localContinuousLQR( ...
        A, B, Q, R);

    closedLoopEigenvalues = ...
        eig(A - B*K);

    lqrMethod = ...
        'Hamiltonian CARE fallback';

end


Acl = A - B*K;


%% ============================================================
% 7. Verify closed-loop stability
% =============================================================

closedLoopEigenvalues = eig(Acl);

isHurwitz = all(real(closedLoopEigenvalues) < 0);


fprintf('\n=============================================\n');
fprintf('Fixed LQR reference controller\n');
fprintf('=============================================\n');

fprintf('LQR method: %s\n', lqrMethod);

fprintf('\nK =\n');
disp(K);

fprintf('Closed-loop eigenvalues:\n');
disp(closedLoopEigenvalues);

fprintf('A-BK Hurwitz: %d\n', isHurwitz);


if ~isHurwitz

    error('The reference closed-loop linearisation is not Hurwitz.');

end


%% ============================================================
% 8. CARE residual check
% =============================================================

careResidual = ...
    A' * P + ...
    P * A - ...
    P * B * (R \ (B' * P)) + ...
    Q;

careResidualNorm = norm(careResidual, 'fro');

fprintf('CARE residual Frobenius norm: %.6e\n', ...
    careResidualNorm);


%% ============================================================
% 9. Save controller reference
% =============================================================

controllerMetadata.system = 'Quadrotor';

controllerMetadata.task = ...
    'Closed-Loop Hover Stability';

controllerMetadata.controller = ...
    'Continuous-time LQR';

controllerMetadata.lqrMethod = lqrMethod;

controllerMetadata.referenceType = ...
    'Analytical + Numerical';

controllerMetadata.stateOrder = ...
    ['px py pz vx vy vz phi theta psi ' ...
     'p q r'];

controllerMetadata.inputOrder = ...
    'T tau_phi tau_theta tau_psi';

controllerMetadata.matlabVersion = version;

controllerMetadata.computer = computer;

controllerMetadata.mass_kg = quad.m;

controllerMetadata.gravity_mps2 = quad.g;

controllerMetadata.inertia = quad.J;

controllerMetadata.hoverState = xEq;

controllerMetadata.hoverInput = uEq;

controllerMetadata.Q = Q;

controllerMetadata.R = R;

controllerMetadata.K = K;

controllerMetadata.closedLoopEigenvalues = ...
    closedLoopEigenvalues;

controllerMetadata.isHurwitz = isHurwitz;

controllerMetadata.controllabilityRank = ...
    controllabilityRank;

controllerMetadata.careResidualNorm = ...
    careResidualNorm;

controllerMetadata.dateGenerated = ...
    char(datetime('now'));


controllerFile = fullfile( ...
    outputFolder, ...
    'QUAD_HOVER_controller_reference.mat');


save(controllerFile, ...
    'A', ...
    'B', ...
    'Q', ...
    'R', ...
    'K', ...
    'P', ...
    'Acl', ...
    'closedLoopEigenvalues', ...
    'controllerMetadata', ...
    'quad', ...
    'xEq', ...
    'uEq');


fprintf('\nSaved controller reference:\n%s\n', ...
    controllerFile);


%% ============================================================
% 10. Benchmark target definition
% =============================================================

target.positionTolerance = 0.05;          % m
target.velocityTolerance = 0.05;          % m/s

target.angleTolerance = deg2rad(2.0);      % rad
target.rateTolerance = 0.05;              % rad/s


%% ============================================================
% 11. Safety region
% =============================================================

safety.xyMaximum = 2.0;                   % m

safety.zMinimum = 0.2;                    % m
safety.zMaximum = 2.5;                    % m

safety.rollPitchMaximum = ...
    deg2rad(45.0);                         % rad


%% ============================================================
% 12. Numerical reference settings
% =============================================================

Tfinal = 10.0;                            % s

RelTol = 1e-10;
AbsTol = 1e-12;
MaxStep = 0.01;                           % s

% Fixed output spacing for metric evaluation
outputDt = 0.001;                         % s

tEval = (0:outputDt:Tfinal)';


odeOptions = odeset( ...
    'RelTol', RelTol, ...
    'AbsTol', AbsTol, ...
    'MaxStep', MaxStep);


%% ============================================================
% 13. Benchmark instances
% =============================================================

% ------------------------------------------------------------
% Baseline
% ------------------------------------------------------------

cases(1).label = 'B';

cases(1).difficulty = ...
    'Baseline';

cases(1).position = [ ...
     0.10; ...
    -0.10; ...
     1.10];

cases(1).anglesDeg = [ ...
     5; ...
    -5; ...
     5];


% ------------------------------------------------------------
% Intermediate
% ------------------------------------------------------------

cases(2).label = 'I';

cases(2).difficulty = ...
    'Intermediate';

cases(2).position = [ ...
     0.25; ...
    -0.25; ...
     1.25];

cases(2).anglesDeg = [ ...
     10; ...
    -10; ...
     10];


% ------------------------------------------------------------
% Challenge
% ------------------------------------------------------------

cases(3).label = 'C';

cases(3).difficulty = ...
    'Challenge';

cases(3).position = [ ...
     0.50; ...
    -0.50; ...
     1.50];

cases(3).anglesDeg = [ ...
     15; ...
    -15; ...
     15];


%% ============================================================
% 14. Initialise summary storage
% =============================================================

summaryDifficulty = strings(3,1);

summarySettlingTime = zeros(3,1);

summaryRecoverySuccess = false(3,1);

summarySafetySatisfied = false(3,1);

summaryComputationTime = zeros(3,1);

summaryPeakPositionError = zeros(3,1);

summaryPeakAttitudeDeg = zeros(3,1);

summaryPeakThrust = zeros(3,1);

summaryPeakTauPhi = zeros(3,1);

summaryPeakTauTheta = zeros(3,1);

summaryPeakTauPsi = zeros(3,1);

summaryControlEffort = zeros(3,1);

summarySaturationFraction = zeros(3,1);


%% ============================================================
% 15. Run B / I / C nonlinear reference simulations
% =============================================================

for kCase = 1:length(cases)

    label = cases(kCase).label;

    difficulty = ...
        cases(kCase).difficulty;


    fprintf('\n');
    fprintf('=============================================\n');
    fprintf('Running Quadrotor Hover Reference: %s\n', ...
        difficulty);
    fprintf('=============================================\n');


    %% --------------------------------------------------------
    % 15.1 Initial condition
    % ---------------------------------------------------------

    x0 = zeros(12,1);

    x0(1:3) = ...
        cases(kCase).position;

    x0(4:6) = [0;0;0];

    x0(7:9) = ...
        deg2rad(cases(kCase).anglesDeg);

    x0(10:12) = [0;0;0];


    fprintf('Initial position [m]:\n');
    disp(x0(1:3)');

    fprintf('Initial Euler angles [deg]:\n');
    disp(cases(kCase).anglesDeg');


    %% --------------------------------------------------------
    % 15.2 Nonlinear simulation
    % ---------------------------------------------------------

    dynamics = @(t,x) ...
        localQuadrotorDynamics( ...
            t, ...
            x, ...
            quad, ...
            K, ...
            xEq, ...
            uEq);


    simulationTimer = tic;


    [t, X] = ode113( ...
        dynamics, ...
        tEval, ...
        x0, ...
        odeOptions);


    computationTime = toc(simulationTimer);


    fprintf('Numerical simulation completed.\n');
    fprintf('Computation time: %.6f s\n', ...
        computationTime);


    %% --------------------------------------------------------
    % 15.3 Reconstruct control input history
    % ---------------------------------------------------------

    numberSamples = length(t);

    U = zeros(numberSamples,4);

    Uunsat = zeros(numberSamples,4);

    saturationActive = false(numberSamples,1);


    for j = 1:numberSamples

        [uSat_j, uUnsat_j, sat_j] = ...
            localQuadrotorController( ...
            X(j,:)', ...
            quad, ...
            K, ...
            xEq, ...
            uEq);

        U(j,:) = uSat_j.';
        Uunsat(j,:) = uUnsat_j.';
        saturationActive(j) = sat_j;

    end


    %% --------------------------------------------------------
    % 15.4 Target-set evaluation
    % ---------------------------------------------------------

    positionError = ...
        X(:,1:3) - xEq(1:3)';

    positionInfinityError = ...
        max(abs(positionError), [], 2);

    positionEuclideanError = ...
        sqrt(sum(positionError.^2, 2));


    velocityInfinity = ...
        max(abs(X(:,4:6)), [], 2);


    attitudeInfinity = ...
        max(abs(X(:,7:9)), [], 2);


    bodyRateInfinity = ...
        max(abs(X(:,10:12)), [], 2);


    insideTarget = ...
        positionInfinityError <= ...
            target.positionTolerance & ...
        velocityInfinity <= ...
            target.velocityTolerance & ...
        attitudeInfinity <= ...
            target.angleTolerance & ...
        bodyRateInfinity <= ...
            target.rateTolerance;


    %% --------------------------------------------------------
    % 15.5 Settling time
    %
    % First time after which the state remains inside
    % the target until the end of the 10 s horizon.
    % ---------------------------------------------------------

    if ~insideTarget(end)

        settlingTime = NaN;

    else

        outsideIndices = ...
            find(~insideTarget);

        if isempty(outsideIndices)

            settlingTime = 0;

        else

            lastOutside = ...
                outsideIndices(end);

            if lastOutside >= length(t)

                settlingTime = NaN;

            else

                settlingTime = ...
                    t(lastOutside + 1);

            end

        end

    end


    %% --------------------------------------------------------
    % 15.6 Safety evaluation
    % ---------------------------------------------------------

    safetyAtTime = ...
        abs(X(:,1)) <= safety.xyMaximum & ...
        abs(X(:,2)) <= safety.xyMaximum & ...
        X(:,3) >= safety.zMinimum & ...
        X(:,3) <= safety.zMaximum & ...
        abs(X(:,7)) <= ...
            safety.rollPitchMaximum & ...
        abs(X(:,8)) <= ...
            safety.rollPitchMaximum;


    safetySatisfied = ...
        all(safetyAtTime);


    %% --------------------------------------------------------
    % 15.7 Recovery success
    % ---------------------------------------------------------

    recoverySuccess = ...
        safetySatisfied && ...
        ~isnan(settlingTime);


    %% --------------------------------------------------------
    % 15.8 Performance metrics
    % ---------------------------------------------------------

    peakPositionError = ...
        max(positionEuclideanError);


    peakAttitudeRad = ...
        max(max(abs(X(:,7:9))));

    peakAttitudeDeg = ...
        rad2deg(peakAttitudeRad);


    peakThrust = ...
        max(U(:,1));

    peakTauPhi = ...
        max(abs(U(:,2)));

    peakTauTheta = ...
        max(abs(U(:,3)));

    peakTauPsi = ...
        max(abs(U(:,4)));


    % Control effort uses deviation from hover input.
    inputDeviation = ...
        U - uEq';

    controlEffort = ...
        trapz(t, ...
        sum(inputDeviation.^2, 2));


    saturationFraction = ...
        mean(saturationActive);


    %% --------------------------------------------------------
    % 15.9 Print reference results
    % ---------------------------------------------------------

    fprintf('\nReference results:\n');

    fprintf('Recovery success: %d\n', ...
        recoverySuccess);

    fprintf('Safety satisfied: %d\n', ...
        safetySatisfied);

    fprintf('Settling time: %.6f s\n', ...
        settlingTime);

    fprintf('Peak Euclidean position error: %.6f m\n', ...
        peakPositionError);

    fprintf('Peak absolute attitude angle: %.6f deg\n', ...
        peakAttitudeDeg);

    fprintf('Peak thrust: %.6f N\n', ...
        peakThrust);

    fprintf('Peak |tau_phi|: %.6f N m\n', ...
        peakTauPhi);

    fprintf('Peak |tau_theta|: %.6f N m\n', ...
        peakTauTheta);

    fprintf('Peak |tau_psi|: %.6f N m\n', ...
        peakTauPsi);

    fprintf('Integrated control effort: %.6f\n', ...
        controlEffort);

    fprintf('Saturation-active fraction: %.6f\n', ...
        saturationFraction);


    %% --------------------------------------------------------
    % 15.10 Reference metadata
    % ---------------------------------------------------------

    metadata.system = 'Quadrotor';

    metadata.task = ...
        'Closed-Loop Hover Stability';

    metadata.benchmarkFamily = ...
        ['Closed-Loop Hover Stability ' ...
         'and Nonlinear Recovery'];

    metadata.difficulty = difficulty;

    metadata.referenceType = ...
        'Analytical + Numerical';

    metadata.controller = ...
        'Fixed continuous-time LQR';

    metadata.controllerGain = K;

    metadata.closedLoopEigenvalues = ...
        closedLoopEigenvalues;

    metadata.mass_kg = quad.m;

    metadata.gravity_mps2 = quad.g;

    metadata.inertia = quad.J;

    metadata.initialState = x0;

    metadata.hoverState = xEq;

    metadata.hoverInput = uEq;

    metadata.timeHorizon_seconds = ...
        Tfinal;

    metadata.RelTol = RelTol;

    metadata.AbsTol = AbsTol;

    metadata.MaxStep_seconds = ...
        MaxStep;

    metadata.outputSampling_seconds = ...
        outputDt;

    metadata.solver = ...
        'MATLAB ode113';

    metadata.matlabVersion = ...
        version;

    metadata.computer = ...
        computer;

    metadata.computationTime_seconds = ...
        computationTime;

    metadata.recoverySuccess = ...
        recoverySuccess;

    metadata.safetySatisfied = ...
        safetySatisfied;

    metadata.settlingTime_seconds = ...
        settlingTime;

    metadata.peakPositionError_m = ...
        peakPositionError;

    metadata.peakAttitude_deg = ...
        peakAttitudeDeg;

    metadata.peakThrust_N = ...
        peakThrust;

    metadata.peakTorque_Nm = [ ...
        peakTauPhi; ...
        peakTauTheta; ...
        peakTauPsi];

    metadata.controlEffort = ...
        controlEffort;

    metadata.saturationActiveFraction = ...
        saturationFraction;

    metadata.dateGenerated = ...
        char(datetime('now'));


    %% --------------------------------------------------------
    % 15.11 Save reference MAT file
    % ---------------------------------------------------------

    referenceFile = fullfile( ...
        outputFolder, ...
        sprintf( ...
        'QUAD_HOVER_%s_reference.mat', ...
        label));


    save(referenceFile, ...
        't', ...
        'X', ...
        'U', ...
        'Uunsat', ...
        'x0', ...
        'xEq', ...
        'uEq', ...
        'A', ...
        'B', ...
        'Q', ...
        'R', ...
        'K', ...
        'Acl', ...
        'closedLoopEigenvalues', ...
        'quad', ...
        'target', ...
        'safety', ...
        'metadata', ...
        '-v7.3');


    fprintf('\nSaved reference file:\n%s\n', ...
        referenceFile);


    %% --------------------------------------------------------
    % 15.12 Save trajectory CSV
    % ---------------------------------------------------------

    trajectoryTable = table( ...
        t, ...
        X(:,1), ...
        X(:,2), ...
        X(:,3), ...
        X(:,4), ...
        X(:,5), ...
        X(:,6), ...
        X(:,7), ...
        X(:,8), ...
        X(:,9), ...
        X(:,10), ...
        X(:,11), ...
        X(:,12), ...
        U(:,1), ...
        U(:,2), ...
        U(:,3), ...
        U(:,4), ...
        insideTarget, ...
        safetyAtTime, ...
        saturationActive, ...
        'VariableNames', { ...
        'time_s', ...
        'px_m', ...
        'py_m', ...
        'pz_m', ...
        'vx_mps', ...
        'vy_mps', ...
        'vz_mps', ...
        'phi_rad', ...
        'theta_rad', ...
        'psi_rad', ...
        'p_radps', ...
        'q_radps', ...
        'r_radps', ...
        'thrust_N', ...
        'tau_phi_Nm', ...
        'tau_theta_Nm', ...
        'tau_psi_Nm', ...
        'inside_target', ...
        'inside_safety_region', ...
        'saturation_active'});


    trajectoryCSV = fullfile( ...
        outputFolder, ...
        sprintf( ...
        'QUAD_HOVER_%s_trajectory.csv', ...
        label));


    writetable( ...
        trajectoryTable, ...
        trajectoryCSV);


    %% --------------------------------------------------------
    % 15.13 Figure: 3-D position trajectory
    % ---------------------------------------------------------

    figure;

    plot3( ...
        X(:,1), ...
        X(:,2), ...
        X(:,3), ...
        'LineWidth', 1.2);

    hold on;

    plot3( ...
        xEq(1), ...
        xEq(2), ...
        xEq(3), ...
        'o', ...
        'MarkerSize', 8, ...
        'LineWidth', 1.5);

    grid on;
    box on;

    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');

    title(sprintf( ...
        'Quadrotor Hover Recovery - %s', ...
        difficulty));


    positionFigure = fullfile( ...
        outputFolder, ...
        sprintf( ...
        'QUAD_HOVER_%s_position.png', ...
        label));


    exportgraphics( ...
        gcf, ...
        positionFigure, ...
        'Resolution', 300);

    close;


    %% --------------------------------------------------------
    % 15.14 Figure: attitude
    % ---------------------------------------------------------

    figure;

    plot( ...
        t, ...
        rad2deg(X(:,7:9)), ...
        'LineWidth', 1.1);

    grid on;
    box on;

    xlabel('Time [s]');
    ylabel('Euler angle [deg]');

    legend( ...
        '\phi', ...
        '\theta', ...
        '\psi', ...
        'Location', ...
        'best');

    title(sprintf( ...
        'Quadrotor Attitude Recovery - %s', ...
        difficulty));


    attitudeFigure = fullfile( ...
        outputFolder, ...
        sprintf( ...
        'QUAD_HOVER_%s_attitude.png', ...
        label));


    exportgraphics( ...
        gcf, ...
        attitudeFigure, ...
        'Resolution', 300);

    close;


    %% --------------------------------------------------------
    % 15.15 Figure: normalised target error
    % ---------------------------------------------------------

    normalisedTargetError = max([ ...
        positionInfinityError ./ ...
            target.positionTolerance, ...
        velocityInfinity ./ ...
            target.velocityTolerance, ...
        attitudeInfinity ./ ...
            target.angleTolerance, ...
        bodyRateInfinity ./ ...
            target.rateTolerance], ...
        [], 2);


    figure;

    plot( ...
        t, ...
        normalisedTargetError, ...
        'LineWidth', 1.2);

    hold on;

    yline( ...
        1, ...
        '--', ...
        'Target-set boundary');

    grid on;
    box on;

    xlabel('Time [s]');

    ylabel( ...
        'Normalised target error');

    title(sprintf( ...
        'Quadrotor Target-Set Recovery - %s', ...
        difficulty));


    targetFigure = fullfile( ...
        outputFolder, ...
        sprintf( ...
        'QUAD_HOVER_%s_target_error.png', ...
        label));


    exportgraphics( ...
        gcf, ...
        targetFigure, ...
        'Resolution', 300);

    close;


    %% --------------------------------------------------------
    % 15.16 Store summary values
    % ---------------------------------------------------------

    summaryDifficulty(kCase) = ...
        string(difficulty);

    summarySettlingTime(kCase) = ...
        settlingTime;

    summaryRecoverySuccess(kCase) = ...
        recoverySuccess;

    summarySafetySatisfied(kCase) = ...
        safetySatisfied;

    summaryComputationTime(kCase) = ...
        computationTime;

    summaryPeakPositionError(kCase) = ...
        peakPositionError;

    summaryPeakAttitudeDeg(kCase) = ...
        peakAttitudeDeg;

    summaryPeakThrust(kCase) = ...
        peakThrust;

    summaryPeakTauPhi(kCase) = ...
        peakTauPhi;

    summaryPeakTauTheta(kCase) = ...
        peakTauTheta;

    summaryPeakTauPsi(kCase) = ...
        peakTauPsi;

    summaryControlEffort(kCase) = ...
        controlEffort;

    summarySaturationFraction(kCase) = ...
        saturationFraction;

end


%% ============================================================
% 16. Generate reference summary table
% =============================================================

summaryTable = table( ...
    summaryDifficulty, ...
    summaryRecoverySuccess, ...
    summarySafetySatisfied, ...
    summarySettlingTime, ...
    summaryPeakPositionError, ...
    summaryPeakAttitudeDeg, ...
    summaryPeakThrust, ...
    summaryPeakTauPhi, ...
    summaryPeakTauTheta, ...
    summaryPeakTauPsi, ...
    summaryControlEffort, ...
    summarySaturationFraction, ...
    summaryComputationTime, ...
    'VariableNames', { ...
    'Difficulty', ...
    'Recovery_Success', ...
    'Safety_Satisfied', ...
    'Settling_Time_s', ...
    'Peak_Position_Error_m', ...
    'Peak_Attitude_deg', ...
    'Peak_Thrust_N', ...
    'Peak_Tau_Phi_Nm', ...
    'Peak_Tau_Theta_Nm', ...
    'Peak_Tau_Psi_Nm', ...
    'Control_Effort', ...
    'Saturation_Fraction', ...
    'Computation_Time_s'});


fprintf('\n');
fprintf('=============================================\n');
fprintf('REFERENCE SUMMARY\n');
fprintf('=============================================\n');

disp(summaryTable);


summaryCSV = fullfile( ...
    outputFolder, ...
    'QUAD_HOVER_reference_summary.csv');


writetable( ...
    summaryTable, ...
    summaryCSV);


save( ...
    fullfile( ...
        outputFolder, ...
        'QUAD_HOVER_reference_summary.mat'), ...
    'summaryTable', ...
    'K', ...
    'closedLoopEigenvalues', ...
    'A', ...
    'B', ...
    'Q', ...
    'R');


%% ============================================================
% 17. Finished
% =============================================================

fprintf('\n');
fprintf('=============================================\n');
fprintf('All Quadrotor hover references completed.\n');
fprintf('=============================================\n');

fprintf('\nGenerated reference files:\n');

fprintf('QUAD_HOVER_B_reference.mat\n');
fprintf('QUAD_HOVER_I_reference.mat\n');
fprintf('QUAD_HOVER_C_reference.mat\n');

fprintf('\nSummary:\n');

fprintf('QUAD_HOVER_reference_summary.csv\n');
fprintf('QUAD_HOVER_controller_reference.mat\n');

fprintf('\nOutput folder:\n');

disp(fullfile(pwd, outputFolder));


%% ============================================================
% LOCAL FUNCTION 1
% Quadrotor nonlinear dynamics
% =============================================================

function dx = localQuadrotorDynamics( ...
    ~, ...
    x, ...
    quad, ...
    K, ...
    xEq, ...
    uEq)


    % --------------------------------------------------------
    % Extract states
    % ---------------------------------------------------------

    velocity = x(4:6);

    phi   = x(7);
    theta = x(8);
    psi   = x(9);

    omega = x(10:12);


    % --------------------------------------------------------
    % Controller
    % ---------------------------------------------------------

    [u,~,~] = ...
        localQuadrotorController( ...
            x, ...
            quad, ...
            K, ...
            xEq, ...
            uEq);


    T = u(1);

    tau = u(2:4);


    % --------------------------------------------------------
    % Rotation matrix Rz * Ry * Rx
    % ---------------------------------------------------------

    cphi = cos(phi);
    sphi = sin(phi);

    ctheta = cos(theta);
    stheta = sin(theta);

    cpsi = cos(psi);
    spsi = sin(psi);


    Rx = [ ...
        1, 0, 0; ...
        0, cphi, -sphi; ...
        0, sphi,  cphi];


    Ry = [ ...
         ctheta, 0, stheta; ...
         0,      1, 0; ...
        -stheta, 0, ctheta];


    Rz = [ ...
        cpsi, -spsi, 0; ...
        spsi,  cpsi, 0; ...
        0,     0,    1];


    Rot = Rz * Ry * Rx;


    % --------------------------------------------------------
    % Position dynamics
    % ---------------------------------------------------------

    positionDot = velocity;


    % --------------------------------------------------------
    % Translational dynamics
    % ---------------------------------------------------------

    e3 = [0;0;1];


    acceleration = ...
        -quad.g * e3 + ...
        (T / quad.m) * Rot * e3;


    % --------------------------------------------------------
    % Euler-angle kinematics
    % ---------------------------------------------------------

    if abs(ctheta) < 1e-6

        error(['Euler-angle singularity encountered: ' ...
               'cos(theta) is too small.']);

    end


    E = [ ...
        1, ...
        sphi*tan(theta), ...
        cphi*tan(theta); ...
        0, ...
        cphi, ...
        -sphi; ...
        0, ...
        sphi/ctheta, ...
        cphi/ctheta];


    etaDot = E * omega;


    % --------------------------------------------------------
    % Rotational dynamics
    % ---------------------------------------------------------

    omegaDot = ...
        quad.J \ ...
        (tau - cross(omega, quad.J*omega));


    % --------------------------------------------------------
    % Complete derivative
    % ---------------------------------------------------------

    dx = [ ...
        positionDot; ...
        acceleration; ...
        etaDot; ...
        omegaDot];

end


%% ============================================================
% LOCAL FUNCTION 2
% Saturated fixed LQR controller
% =============================================================

function [u, uUnsat, saturationActive] = ...
    localQuadrotorController( ...
        x, ...
        quad, ...
        K, ...
        xEq, ...
        uEq)


    stateError = x - xEq;


    % LQR input deviation
    deltaU = -K * stateError;


    % Convert thrust deviation into total thrust
    uUnsat = uEq + deltaU;


    % --------------------------------------------------------
    % Saturation
    % ---------------------------------------------------------

    u = uUnsat;


    % Thrust
    u(1) = min( ...
        max(u(1), quad.Tmin), ...
        quad.Tmax);


    % Roll torque
    u(2) = min( ...
        max(u(2), -quad.tauMax(1)), ...
        quad.tauMax(1));


    % Pitch torque
    u(3) = min( ...
        max(u(3), -quad.tauMax(2)), ...
        quad.tauMax(2));


    % Yaw torque
    u(4) = min( ...
        max(u(4), -quad.tauMax(3)), ...
        quad.tauMax(3));


    saturationActive = ...
        any(abs(u - uUnsat) > 1e-12);

end


%% ============================================================
% LOCAL FUNCTION 3
% Continuous-time LQR fallback if lqr() is unavailable
% =============================================================

function [K,P] = ...
    localContinuousLQR(A,B,Q,R)


    n = size(A,1);


    % Hamiltonian matrix
    H = [ ...
        A, ...
        -B*(R\B'); ...
        -Q, ...
        -A'];


    [V,D] = eig(H);

    lambda = diag(D);


    stableIndex = ...
        find(real(lambda) < 0);


    if length(stableIndex) ~= n

        error(['Hamiltonian method could not identify ' ...
               'exactly n stable eigenvalues.']);

    end


    Vs = V(:,stableIndex);


    V1 = Vs(1:n,:);

    V2 = Vs(n+1:end,:);


    if rcond(V1) < 1e-12

        error(['Hamiltonian invariant subspace is ' ...
               'numerically ill-conditioned.']);

    end


    P = real(V2 / V1);


    % Enforce numerical symmetry
    P = 0.5 * (P + P');


    K = R \ (B' * P);

end