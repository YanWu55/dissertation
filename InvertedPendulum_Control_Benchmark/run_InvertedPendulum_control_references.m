function summaryTable = run_InvertedPendulum_control_references()
%RUN_INVERTEDPENDULUM_CONTROL_REFERENCES
% A.9 Inverted Pendulum — Upright Stabilisation
%
% Target environment:
%   MATLAB R2026a
%
% Benchmark plant:
%   x = [theta; omega]
%   theta = 0 is the upright equilibrium
%
%   theta_dot = omega
%   omega_dot = (g/l)*sin(theta) ...
%             - (b/(m*l^2))*omega ...
%             + u/(m*l^2)
%
% Fixed parameters:
%   m = 1.0 kg
%   l = 1.0 m
%   b = 0.1 N m s/rad
%   g = 9.81 m/s^2
%
% Input constraint:
%   |u| <= 10 N m
%
% Fixed continuous-time LQR:
%   Q = diag([10 1])
%   R = 1
%
% Benchmark instances:
%   Baseline      theta0 =  5 deg, omega0 = 0
%   Intermediate  theta0 = 15 deg, omega0 = 0
%   Challenge     theta0 = 30 deg, omega0 = 0
%
% Target neighbourhood:
%   |theta| <= 1 deg
%   |omega| <= 0.05 rad/s
%
% Settling time:
%   Earliest time after which BOTH target conditions remain satisfied
%   for the rest of the 10 s simulation.
%
% Primary numerical reference:
%   ode113, RelTol = 1e-10, AbsTol = 1e-12, MaxStep = 0.001 s
%
% Independent consistency check:
%   ode45 with the same fixed output grid and tight tolerances.
%
% Outputs:
%   InvertedPendulum_Control_References/
%       INVPEND_CTRL_controller_reference.mat
%       INVPEND_CTRL_B_reference.mat
%       INVPEND_CTRL_I_reference.mat
%       INVPEND_CTRL_C_reference.mat
%       INVPEND_CTRL_B_trajectory.csv
%       INVPEND_CTRL_I_trajectory.csv
%       INVPEND_CTRL_C_trajectory.csv
%       INVPEND_CTRL_reference_summary.csv
%       INVPEND_CTRL_reference_summary.mat
%
% NOTE:
%   This benchmark is Analytical + Numerical:
%   - analytical: upright linearisation and fixed LQR design;
%   - numerical: nonlinear saturated closed-loop trajectories.

clc;
close all;

%% ========================================================================
% 0. Environment checks
% =========================================================================

fprintf('MATLAB version: %s\n', version);
fprintf('Computer platform: %s\n', computer);

if isempty(which('lqr'))
    error(['lqr() was not found. Control System Toolbox is required ', ...
        'for this reference generator.']);
end

%% ========================================================================
% 1. Fixed benchmark parameters
% =========================================================================

p.m = 1.0;      % kg
p.l = 1.0;      % m
p.b = 0.1;      % N m s/rad
p.g = 9.81;     % m/s^2

p.uMin = -10.0; % N m
p.uMax =  10.0; % N m

p.thetaTarget = deg2rad(1.0);
p.omegaTarget = 0.05;

p.tFinal = 10.0;
p.outputStep = 0.001;

fprintf('\n============================================================\n');
fprintf('A.9 Inverted Pendulum — Upright Stabilisation\n');
fprintf('============================================================\n');
fprintf('State order: [theta omega]\n');
fprintf('m = %.6f kg\n', p.m);
fprintf('l = %.6f m\n', p.l);
fprintf('b = %.6f N m s/rad\n', p.b);
fprintf('g = %.6f m/s^2\n', p.g);
fprintf('Torque constraint: [%.3f, %.3f] N m\n', p.uMin, p.uMax);
fprintf('Target: |theta| <= %.3f deg, |omega| <= %.3f rad/s\n', ...
    rad2deg(p.thetaTarget), p.omegaTarget);
fprintf('Simulation horizon: %.3f s\n', p.tFinal);

outputFolder = 'InvertedPendulum_Control_References';
if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end

%% ========================================================================
% 2. Upright linearisation and fixed LQR
% =========================================================================

% x = [theta; omega], theta = 0 upright.
A = [ ...
    0, 1; ...
    p.g/p.l, -p.b/(p.m*p.l^2)];

B = [ ...
    0; ...
    1/(p.m*p.l^2)];

controllabilityRank = rank(ctrb(A,B));

Q = diag([10,1]);
R = 1;

[K,P,closedLoopEigenvalues] = lqr(A,B,Q,R);
Acl = A-B*K;

isHurwitz = all(real(closedLoopEigenvalues) < 0);

careResidual = A'*P + P*A - P*B*(R\(B'*P)) + Q;
careResidualNorm = norm(careResidual,'fro');

fprintf('\n============================================================\n');
fprintf('Fixed LQR reference controller\n');
fprintf('============================================================\n');
fprintf('Controllability rank: %d / 2\n', controllabilityRank);
fprintf('Q = diag([10 1])\n');
fprintf('R = 1\n');

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

fprintf('A-BK Hurwitz: %d\n', isHurwitz);
fprintf('CARE residual Frobenius norm: %.6e\n', careResidualNorm);

if controllabilityRank < 2
    error('The upright linearised system is not controllable.');
end

if ~isHurwitz
    error('The fixed LQR closed loop is not Hurwitz.');
end

%% ========================================================================
% 3. Save controller reference
% =========================================================================

controllerMetadata.system = 'Inverted Pendulum';
controllerMetadata.task = 'Control Design';
controllerMetadata.benchmarkFamily = 'Upright Stabilisation';
controllerMetadata.benchmarkRole = 'Baseline / Correctness';
controllerMetadata.referenceType = 'Analytical + Numerical';

controllerMetadata.stateOrder = 'theta omega';
controllerMetadata.angleConvention = 'theta = 0 is upright';

controllerMetadata.mass_kg = p.m;
controllerMetadata.length_m = p.l;
controllerMetadata.damping_Nm_s_per_rad = p.b;
controllerMetadata.gravity_mps2 = p.g;

controllerMetadata.torqueMin_Nm = p.uMin;
controllerMetadata.torqueMax_Nm = p.uMax;

controllerMetadata.Q = Q;
controllerMetadata.R = R;
controllerMetadata.K = K;
controllerMetadata.P = P;
controllerMetadata.A = A;
controllerMetadata.B = B;
controllerMetadata.closedLoopEigenvalues = closedLoopEigenvalues;
controllerMetadata.controllabilityRank = controllabilityRank;
controllerMetadata.careResidualNorm = careResidualNorm;

controllerMetadata.targetTheta_deg = rad2deg(p.thetaTarget);
controllerMetadata.targetOmega_radps = p.omegaTarget;
controllerMetadata.simulationHorizon_s = p.tFinal;

controllerMetadata.matlabVersion = version;
controllerMetadata.computer = computer;
controllerMetadata.dateGenerated = char(datetime('now'));

save( ...
    fullfile(outputFolder,'INVPEND_CTRL_controller_reference.mat'), ...
    'A','B','Q','R','K','P','Acl','closedLoopEigenvalues', ...
    'controllerMetadata','p');

%% ========================================================================
% 4. Benchmark instances
% =========================================================================

cases(1).label = 'B';
cases(1).difficulty = 'Baseline';
cases(1).theta0_deg = 5;

cases(2).label = 'I';
cases(2).difficulty = 'Intermediate';
cases(2).theta0_deg = 15;

cases(3).label = 'C';
cases(3).difficulty = 'Challenge';
cases(3).theta0_deg = 30;

%% ========================================================================
% 5. Numerical solver configuration
% =========================================================================

tEval = (0:p.outputStep:p.tFinal)';

primaryOptions = odeset( ...
    'RelTol',1e-10, ...
    'AbsTol',1e-12, ...
    'MaxStep',0.001);

validationOptions = odeset( ...
    'RelTol',1e-11, ...
    'AbsTol',1e-13, ...
    'MaxStep',0.001);

%% ========================================================================
% 6. Summary storage
% =========================================================================

nCases = numel(cases);

Difficulty = strings(nCases,1);
Initial_Angle_deg = zeros(nCases,1);

Recovery_Success = false(nCases,1);
Settling_Time_s = nan(nCases,1);

Peak_Torque_Nm = nan(nCases,1);
Control_Effort_Nm2s = nan(nCases,1);
Maximum_Angle_deg = nan(nCases,1);
Maximum_Angular_Velocity_radps = nan(nCases,1);

Final_Angle_deg = nan(nCases,1);
Final_Angular_Velocity_radps = nan(nCases,1);
Final_State_Error = nan(nCases,1);

Saturation_Fraction = nan(nCases,1);
Constraint_Satisfied = false(nCases,1);

Ode113_vs_Ode45_Final_Error = nan(nCases,1);
Computation_Time_s = nan(nCases,1);

%% ========================================================================
% 7. Run Baseline / Intermediate / Challenge
% =========================================================================

for kCase = 1:nCases

    label = cases(kCase).label;
    difficulty = cases(kCase).difficulty;
    theta0_deg = cases(kCase).theta0_deg;

    x0 = [deg2rad(theta0_deg); 0];

    fprintf('\n============================================================\n');
    fprintf('Running Inverted Pendulum Control: %s\n',difficulty);
    fprintf('Initial state: theta0 = %.3f deg, omega0 = 0 rad/s\n', ...
        theta0_deg);
    fprintf('============================================================\n');

    %% --------------------------------------------------------------------
    % 7.1 Primary high-accuracy nonlinear simulation with ode113
    % ---------------------------------------------------------------------

    closedLoopODE = @(t,x) localClosedLoopDynamics(x,K,p); %#ok<INUSD>

    simulationTimer = tic;

    [t,x] = ode113( ...
        closedLoopODE, ...
        tEval, ...
        x0, ...
        primaryOptions);

    computationTime = toc(simulationTimer);

    solverSuccess = ...
        numel(t) == numel(tEval) && ...
        all(isfinite(x(:)));

    if ~solverSuccess
        error('Primary ode113 simulation failed for %s.',difficulty);
    end

    %% --------------------------------------------------------------------
    % 7.2 Reconstruct applied and unsaturated torques
    % ---------------------------------------------------------------------

    nSamples = numel(t);

    uApplied = zeros(nSamples,1);
    uUnsaturated = zeros(nSamples,1);

    for j = 1:nSamples
        [uApplied(j),uUnsaturated(j)] = ...
            localController(x(j,:)',K,p);
    end

    %% --------------------------------------------------------------------
    % 7.3 Settling / success metrics
    % ---------------------------------------------------------------------

    theta = x(:,1);
    omega = x(:,2);

    insideTarget = ...
        abs(theta) <= p.thetaTarget & ...
        abs(omega) <= p.omegaTarget;

    settlingTime = localSettlingTime(t,insideTarget);

    recoverySuccess = isfinite(settlingTime);

    peakTorque = max(abs(uApplied));
    controlEffort = trapz(t,uApplied.^2);

    maxAngleDeg = max(abs(rad2deg(theta)));
    maxOmega = max(abs(omega));

    finalAngleDeg = rad2deg(theta(end));
    finalOmega = omega(end);
    finalStateError = norm(x(end,:)',2);

    inputViolation = max([ ...
        max(uApplied-p.uMax), ...
        max(p.uMin-uApplied), ...
        0]);

    constraintSatisfied = inputViolation <= 1e-10;

    saturationActive = ...
        abs(uUnsaturated) >= max(abs([p.uMin,p.uMax])) - 1e-10;

    saturationFraction = mean(saturationActive);

    referenceSuccess = ...
        solverSuccess && ...
        recoverySuccess && ...
        constraintSatisfied;

    %% --------------------------------------------------------------------
    % 7.4 Independent ode45 consistency check
    % ---------------------------------------------------------------------

    [tCheck,xCheck] = ode45( ...
        closedLoopODE, ...
        tEval, ...
        x0, ...
        validationOptions);

    validationSuccess = ...
        numel(tCheck) == numel(tEval) && ...
        all(isfinite(xCheck(:)));

    if ~validationSuccess
        error('Independent ode45 validation failed for %s.',difficulty);
    end

    finalCrossSolverError = ...
        norm(x(end,:)'-xCheck(end,:)',2);

    %% --------------------------------------------------------------------
    % 7.5 Display reference result
    % ---------------------------------------------------------------------

    fprintf('\nREFERENCE RESULT: %s\n',difficulty);
    fprintf('Reference success: %d\n',referenceSuccess);
    fprintf('Settling time: %.6f s\n',settlingTime);
    fprintf('Maximum |theta|: %.6f deg\n',maxAngleDeg);
    fprintf('Maximum |omega|: %.6f rad/s\n',maxOmega);
    fprintf('Peak |u|: %.6f N m\n',peakTorque);
    fprintf('Control effort integral: %.6f N^2 m^2 s\n',controlEffort);
    fprintf('Saturation-active fraction: %.6f\n',saturationFraction);
    fprintf('Final theta: %.9e deg\n',finalAngleDeg);
    fprintf('Final omega: %.9e rad/s\n',finalOmega);
    fprintf('Final-state norm: %.9e\n',finalStateError);
    fprintf('ode113-vs-ode45 final-state difference: %.9e\n', ...
        finalCrossSolverError);
    fprintf('Computation time: %.6f s\n',computationTime);

    %% --------------------------------------------------------------------
    % 7.6 Metadata
    % ---------------------------------------------------------------------

    metadata.system = 'Inverted Pendulum';
    metadata.task = 'Control Design';
    metadata.benchmarkFamily = 'Upright Stabilisation';
    metadata.benchmarkRole = 'Baseline / Correctness';
    metadata.referenceType = 'Analytical + Numerical';

    metadata.difficulty = difficulty;
    metadata.stateOrder = 'theta omega';
    metadata.angleConvention = 'theta = 0 is upright';

    metadata.initialState = x0;
    metadata.initialAngle_deg = theta0_deg;
    metadata.initialAngularVelocity_radps = 0;

    metadata.mass_kg = p.m;
    metadata.length_m = p.l;
    metadata.damping_Nm_s_per_rad = p.b;
    metadata.gravity_mps2 = p.g;

    metadata.torqueMin_Nm = p.uMin;
    metadata.torqueMax_Nm = p.uMax;

    metadata.Q = Q;
    metadata.R = R;
    metadata.K = K;
    metadata.P = P;
    metadata.A = A;
    metadata.B = B;
    metadata.closedLoopEigenvalues = closedLoopEigenvalues;

    metadata.targetTheta_deg = rad2deg(p.thetaTarget);
    metadata.targetOmega_radps = p.omegaTarget;
    metadata.simulationHorizon_s = p.tFinal;

    metadata.primarySolver = 'MATLAB ode113';
    metadata.primaryRelTol = 1e-10;
    metadata.primaryAbsTol = 1e-12;
    metadata.primaryMaxStep_s = 0.001;
    metadata.outputStep_s = p.outputStep;

    metadata.validationSolver = 'MATLAB ode45';
    metadata.validationRelTol = 1e-11;
    metadata.validationAbsTol = 1e-13;
    metadata.validationMaxStep_s = 0.001;

    metadata.referenceSuccess = referenceSuccess;
    metadata.settlingTime_s = settlingTime;
    metadata.maximumAngle_deg = maxAngleDeg;
    metadata.maximumAngularVelocity_radps = maxOmega;
    metadata.peakTorque_Nm = peakTorque;
    metadata.controlEffort_Nm2s = controlEffort;
    metadata.saturationFraction = saturationFraction;
    metadata.finalAngle_deg = finalAngleDeg;
    metadata.finalAngularVelocity_radps = finalOmega;
    metadata.finalStateError = finalStateError;
    metadata.constraintSatisfied = constraintSatisfied;
    metadata.ode113_vs_ode45_finalError = finalCrossSolverError;
    metadata.computationTime_seconds = computationTime;

    metadata.matlabVersion = version;
    metadata.computer = computer;
    metadata.dateGenerated = char(datetime('now'));

    %% --------------------------------------------------------------------
    % 7.7 Save MAT reference
    % ---------------------------------------------------------------------

    referenceFile = fullfile( ...
        outputFolder, ...
        sprintf('INVPEND_CTRL_%s_reference.mat',label));

    save( ...
        referenceFile, ...
        't','x','uApplied','uUnsaturated', ...
        'tCheck','xCheck', ...
        'metadata','A','B','Q','R','K','P','p','-v7.3');

    fprintf('\nSaved reference file:\n%s\n',referenceFile);

    %% --------------------------------------------------------------------
    % 7.8 Save trajectory CSV
    % ---------------------------------------------------------------------

    trajectoryTable = table( ...
        t, ...
        rad2deg(theta), ...
        omega, ...
        uApplied, ...
        uUnsaturated, ...
        'VariableNames',{ ...
            'Time_s', ...
            'Theta_deg', ...
            'Omega_radps', ...
            'Applied_Torque_Nm', ...
            'Unsaturated_Torque_Nm'});

    trajectoryFile = fullfile( ...
        outputFolder, ...
        sprintf('INVPEND_CTRL_%s_trajectory.csv',label));

    writetable(trajectoryTable,trajectoryFile);

    %% --------------------------------------------------------------------
    % 7.9 Summary storage
    % ---------------------------------------------------------------------

    Difficulty(kCase) = string(difficulty);
    Initial_Angle_deg(kCase) = theta0_deg;

    Recovery_Success(kCase) = referenceSuccess;
    Settling_Time_s(kCase) = settlingTime;

    Peak_Torque_Nm(kCase) = peakTorque;
    Control_Effort_Nm2s(kCase) = controlEffort;
    Maximum_Angle_deg(kCase) = maxAngleDeg;
    Maximum_Angular_Velocity_radps(kCase) = maxOmega;

    Final_Angle_deg(kCase) = finalAngleDeg;
    Final_Angular_Velocity_radps(kCase) = finalOmega;
    Final_State_Error(kCase) = finalStateError;

    Saturation_Fraction(kCase) = saturationFraction;
    Constraint_Satisfied(kCase) = constraintSatisfied;

    Ode113_vs_Ode45_Final_Error(kCase) = finalCrossSolverError;
    Computation_Time_s(kCase) = computationTime;
end

%% ========================================================================
% 8. Final summary
% =========================================================================

summaryTable = table( ...
    Difficulty, ...
    Initial_Angle_deg, ...
    Recovery_Success, ...
    Settling_Time_s, ...
    Maximum_Angle_deg, ...
    Maximum_Angular_Velocity_radps, ...
    Peak_Torque_Nm, ...
    Control_Effort_Nm2s, ...
    Saturation_Fraction, ...
    Final_Angle_deg, ...
    Final_Angular_Velocity_radps, ...
    Final_State_Error, ...
    Constraint_Satisfied, ...
    Ode113_vs_Ode45_Final_Error, ...
    Computation_Time_s);

fprintf('\n============================================================\n');
fprintf('REFERENCE SUMMARY\n');
fprintf('============================================================\n');
disp(summaryTable);

writetable( ...
    summaryTable, ...
    fullfile(outputFolder,'INVPEND_CTRL_reference_summary.csv'));

save( ...
    fullfile(outputFolder,'INVPEND_CTRL_reference_summary.mat'), ...
    'summaryTable','A','B','Q','R','K','P','p');

fprintf('\n============================================================\n');
fprintf('All Inverted Pendulum references completed.\n');
fprintf('============================================================\n');

fprintf('\nGenerated reference files:\n');
fprintf('INVPEND_CTRL_B_reference.mat\n');
fprintf('INVPEND_CTRL_I_reference.mat\n');
fprintf('INVPEND_CTRL_C_reference.mat\n');

fprintf('\nTrajectory files:\n');
fprintf('INVPEND_CTRL_B_trajectory.csv\n');
fprintf('INVPEND_CTRL_I_trajectory.csv\n');
fprintf('INVPEND_CTRL_C_trajectory.csv\n');

fprintf('\nSummary:\n');
fprintf('INVPEND_CTRL_reference_summary.csv\n');

fprintf('\nOutput folder:\n%s\n',fullfile(pwd,outputFolder));

end


%% =========================================================================
% LOCAL FUNCTION: saturated LQR controller
% =========================================================================

function [uApplied,uUnsaturated] = localController(x,K,p)

uUnsaturated = -K*x;

uApplied = min(max(uUnsaturated,p.uMin),p.uMax);

end


%% =========================================================================
% LOCAL FUNCTION: nonlinear saturated closed-loop dynamics
% =========================================================================

function dx = localClosedLoopDynamics(x,K,p)

theta = x(1);
omega = x(2);

[uApplied,~] = localController(x,K,p);

thetaDot = omega;

omegaDot = ...
    (p.g/p.l)*sin(theta) ...
    - (p.b/(p.m*p.l^2))*omega ...
    + uApplied/(p.m*p.l^2);

dx = [thetaDot;omegaDot];

end


%% =========================================================================
% LOCAL FUNCTION: settling time
%
% Earliest time after which the state remains inside the target set for
% every subsequent sampled time.
% =========================================================================

function settlingTime = localSettlingTime(t,insideTarget)

insideTarget = logical(insideTarget(:));

% Reverse cumulative logical AND.
remainsInside = flipud(cumprod(double(flipud(insideTarget)))) > 0;

firstIndex = find(remainsInside,1,'first');

if isempty(firstIndex)
    settlingTime = NaN;
else
    settlingTime = t(firstIndex);
end

end
