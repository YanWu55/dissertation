%% Cart-Pole Open-Loop Reachability Reference Generation
%
% Benchmark:
%   A.6.3 Cart-Pole - Open-Loop Unstable Reachability
%
% State:
%   x(1) = p      cart position [m]
%   x(2) = v      cart velocity [m/s]
%   x(3) = theta  pole angle from upright [rad]
%   x(4) = omega  pole angular velocity [rad/s]
%
% Input:
%   u = horizontal cart force [N]
%
% For this reachability benchmark:
%   u(t) = 0 exactly
%
% Fixed parameters:
%   M = 1.0 kg
%   m = 0.1 kg
%   l = 0.5 m
%   g = 9.81 m/s^2
%
% Benchmark horizon:
%   T = 1.0 s for all three difficulty levels
%
% Generates:
%   CARTPOLE_REACH_B_reference.mat
%   CARTPOLE_REACH_I_reference.mat
%   CARTPOLE_REACH_C_reference.mat
%
% plus:
%   validation trajectory files
%   projected flowpipe figures
%   summary metadata
%
% IMPORTANT:
% The sampled trajectories are independent numerical consistency
% checks only. They do NOT replace the CORA set-based reference.


clear;
clc;
close all;


%% ============================================================
% 0. Check CORA installation
% =============================================================

if isempty(which('nonlinearSys'))

    error(['CORA cannot be found on the MATLAB path. ' ...
           'Add the CORA root directory before running this script.']);

end


fprintf('MATLAB version: %s\n', version);
fprintf('Computer platform: %s\n', computer);

try
    fprintf('CORA version: %s\n', CORAVERSION);
catch
    warning('CORA version could not be read automatically.');
end


%% ============================================================
% 1. Output folder
% =============================================================

outputFolder = ...
    'CartPole_Reachability_References';


if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end


%% ============================================================
% 2. Fixed Cart-Pole parameters
% =============================================================

M = 1.0;       % cart mass [kg]
m = 0.1;       % pole mass [kg]
l = 0.5;       % pole length [m]
g = 9.81;      % gravity [m/s^2]

Tfinal = 1.0;  % fixed benchmark horizon [s]


fprintf('\n');
fprintf('=============================================\n');
fprintf('Cart-Pole Reachability Benchmark\n');
fprintf('=============================================\n');

fprintf('Cart mass M   = %.6f kg\n', M);
fprintf('Pole mass m   = %.6f kg\n', m);
fprintf('Pole length l = %.6f m\n', l);
fprintf('Gravity g     = %.6f m/s^2\n', g);
fprintf('Horizon       = %.6f s\n', Tfinal);
fprintf('Input force   = 0 N\n');


%% ============================================================
% 3. Nonlinear Cart-Pole dynamics
%
% Angle theta is measured from the upright configuration.
%
% p_dot     = v
%
% p_ddot =
%
%   u - m*g*sin(theta)*cos(theta)
%     + m*l*omega^2*sin(theta)
%   --------------------------------
%        M + m*sin(theta)^2
%
% theta_dot = omega
%
% theta_ddot =
%
%   (g*sin(theta) - p_ddot*cos(theta))/l
%
% A dummy input variable is retained because CORA nonlinearSys
% uses the standard form xdot = f(x,u).
% =============================================================

f = @(x,u) localCartPoleDynamics( ...
    x, ...
    u, ...
    M, ...
    m, ...
    l, ...
    g);


sys = nonlinearSys( ...
    'CartPoleOpenLoop', ...
    f, ...
    4, ...
    1);


%% ============================================================
% 4. Basic dynamics sanity checks
% =============================================================

testUpright = ...
    f([0;0;0;0],0);


fprintf('\nDynamics at upright equilibrium with u=0:\n');
disp(testUpright);


if norm(testUpright,inf) > 1e-12

    error(['The implemented dynamics do not preserve the ' ...
           'upright equilibrium at zero input.']);

end


testValue = ...
    f([0.1;0.0;deg2rad(5);0.0],0);


if ~isequal(size(testValue), [4 1])

    error('Cart-Pole dynamics do not return a 4x1 column vector.');

end


fprintf('Dynamics dimension check passed.\n');


%% ============================================================
% 5. Common CORA configuration
% =============================================================

params.tStart = 0;
params.tFinal = Tfinal;


% ------------------------------------------------------------
% Exact zero input
% ------------------------------------------------------------

params.U = zonotope(0,0);


% ------------------------------------------------------------
% Nonlinear reachability algorithm
% ------------------------------------------------------------

options.alg = 'poly-adaptive';


%% ============================================================
% 6. Difficulty-level initial sets
%
% Baseline:
%   p     +/- 0.01 m
%   v     +/- 0.01 m/s
%   theta +/- 1 deg
%   omega +/- 0.01 rad/s
%
% Intermediate:
%   p     +/- 0.05
%   v     +/- 0.05
%   theta +/- 3 deg
%   omega +/- 0.05
%
% Challenge:
%   p     +/- 0.10
%   v     +/- 0.10
%   theta +/- 5 deg
%   omega +/- 0.10
% =============================================================

cases(1).label = 'B';

cases(1).difficulty = ...
    'Baseline';

cases(1).lb = [ ...
    -0.01; ...
    -0.01; ...
    -deg2rad(1); ...
    -0.01];

cases(1).ub = [ ...
     0.01; ...
     0.01; ...
     deg2rad(1); ...
     0.01];


cases(2).label = 'I';

cases(2).difficulty = ...
    'Intermediate';

cases(2).lb = [ ...
    -0.05; ...
    -0.05; ...
    -deg2rad(3); ...
    -0.05];

cases(2).ub = [ ...
     0.05; ...
     0.05; ...
     deg2rad(3); ...
     0.05];


cases(3).label = 'C';

cases(3).difficulty = ...
    'Challenge';

cases(3).lb = [ ...
    -0.10; ...
    -0.10; ...
    -deg2rad(5); ...
    -0.10];

cases(3).ub = [ ...
     0.10; ...
     0.10; ...
     deg2rad(5); ...
     0.10];


%% ============================================================
% 7. Independent trajectory-validation configuration
% =============================================================

numberValidationTrajectories = 100;


% Fixed seed for reproducibility
rng(2026, 'twister');


validationRelTol = 1e-12;
validationAbsTol = 1e-14;
validationMaxStep = 1e-3;


odeOptions = odeset( ...
    'RelTol', validationRelTol, ...
    'AbsTol', validationAbsTol, ...
    'MaxStep', validationMaxStep);


% Fixed output grid over 1 second
outputDt = 0.001;

tEval = ...
    (0:outputDt:Tfinal)';


%% ============================================================
% 8. Summary storage
% =============================================================

summaryDifficulty = strings(3,1);

summaryComputationTime = zeros(3,1);

summaryInitialThetaDeg = zeros(3,1);

summaryValidationCount = zeros(3,1);


%% ============================================================
% 9. Run B / I / C reachability computations
% =============================================================

for k = 1:length(cases)

    label = ...
        cases(k).label;

    difficulty = ...
        cases(k).difficulty;

    lb = ...
        cases(k).lb;

    ub = ...
        cases(k).ub;


    fprintf('\n');
    fprintf('=============================================\n');
    fprintf('Running %s reference (%s)\n', ...
        label, difficulty);

    fprintf('=============================================\n');


    %% --------------------------------------------------------
    % 9.1 Initial interval set
    % ---------------------------------------------------------

    X0_interval = ...
        interval(lb,ub);


    params.R0 = ...
        polyZonotope(X0_interval);


    fprintf('Initial lower bound:\n');
    disp(lb');

    fprintf('Initial upper bound:\n');
    disp(ub');


    %% --------------------------------------------------------
    % 9.2 CORA reachability computation
    % ---------------------------------------------------------

    try

        reachTimer = tic;


        R = reach( ...
            sys, ...
            params, ...
            options);


        computationTime = ...
            toc(reachTimer);


        fprintf('\n');
        fprintf('CORA reachability completed successfully.\n');

        fprintf('Computation time: %.6f s\n', ...
            computationTime);


    catch ME

        fprintf('\n');
        fprintf('ERROR during %s reachability computation.\n', ...
            difficulty);

        fprintf('%s\n', ...
            ME.message);


        rethrow(ME);

    end


    %% ========================================================
    % 10. Independent high-accuracy validation trajectories
    % =========================================================

    fprintf('\n');
    fprintf('Generating %d validation trajectories...\n', ...
        numberValidationTrajectories);


    validationTrajectories = ...
        cell(numberValidationTrajectories,1);


    validationInitialStates = ...
        zeros(4,numberValidationTrajectories);


    cartPoleODE = @(t,x) ...
        localCartPoleDynamics( ...
            x, ...
            0, ...
            M, ...
            m, ...
            l, ...
            g);


    for j = 1:numberValidationTrajectories

        % Uniform initial sample from the complete initial box
        x0 = ...
            lb + ...
            (ub-lb).*rand(4,1);


        validationInitialStates(:,j) = ...
            x0;


        [tTrajectory,xTrajectory] = ...
            ode113( ...
                cartPoleODE, ...
                tEval, ...
                x0, ...
                odeOptions);


        validationTrajectories{j}.t = ...
            tTrajectory;

        validationTrajectories{j}.x = ...
            xTrajectory;

    end


    fprintf('Validation trajectories completed.\n');


    %% ========================================================
    % 11. Reproducibility metadata
    % =========================================================

    metadata.system = ...
        'Cart-Pole';

    metadata.task = ...
        'Reachability';

    metadata.benchmarkFamily = ...
        'Open-Loop Unstable Reachability';

    metadata.difficulty = ...
        difficulty;

    metadata.stateOrder = ...
        'p v theta omega';

    metadata.angleConvention = ...
        'theta = 0 is upright';

    metadata.cartMass_kg = ...
        M;

    metadata.poleMass_kg = ...
        m;

    metadata.poleLength_m = ...
        l;

    metadata.gravity_mps2 = ...
        g;

    metadata.inputForce_N = ...
        0;

    metadata.initialLowerBound = ...
        lb;

    metadata.initialUpperBound = ...
        ub;

    metadata.tStart = ...
        0;

    metadata.tFinal = ...
        Tfinal;

    metadata.algorithm = ...
        'poly-adaptive';

    metadata.initialSetRepresentation = ...
        'Polynomial zonotope';

    metadata.inputSetRepresentation = ...
        'Exact zero-input zonotope';

    metadata.referenceType = ...
        'Numerical - set-based reachability reference';

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
        outputDt;

    metadata.randomSeed = ...
        2026;

    metadata.matlabVersion = ...
        version;

    metadata.computer = ...
        computer;

    try

        metadata.coraVersion = ...
            CORAVERSION;

    catch

        metadata.coraVersion = ...
            'CORA version not read automatically';

    end


    metadata.dateGenerated = ...
        char(datetime('now'));


    %% ========================================================
    % 12. Save CORA reference object
    % =========================================================

    referenceFile = fullfile( ...
        outputFolder, ...
        sprintf( ...
            'CARTPOLE_REACH_%s_reference.mat', ...
            label));


    save( ...
        referenceFile, ...
        'R', ...
        'sys', ...
        'params', ...
        'options', ...
        'metadata', ...
        'M', ...
        'm', ...
        'l', ...
        'g', ...
        'lb', ...
        'ub', ...
        '-v7.3');


    fprintf('\nSaved reference file:\n');
    fprintf('%s\n', referenceFile);


    %% ========================================================
    % 13. Save independent validation trajectories
    % =========================================================

    validationMetadata.description = ...
        ['Independent high-accuracy sampled trajectories used ' ...
         'as a numerical consistency check. These trajectories ' ...
         'do not replace the set-based CORA reference.'];


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
        outputDt;

    validationMetadata.randomSeed = ...
        2026;


    validationFile = fullfile( ...
        outputFolder, ...
        sprintf( ...
            'CARTPOLE_REACH_%s_validation.mat', ...
            label));


    save( ...
        validationFile, ...
        'validationTrajectories', ...
        'validationInitialStates', ...
        'validationMetadata', ...
        'tEval', ...
        '-v7.3');


    fprintf('Saved validation file:\n');
    fprintf('%s\n', validationFile);


    %% ========================================================
    % 14. Plot projection: cart position vs pole angle
    % =========================================================

    figure;

    hold on;
    box on;
    grid on;


    plot(R,[1 3]);

    plot(params.R0,[1 3]);


    for j = 1:numberValidationTrajectories

        trajectory = ...
            validationTrajectories{j}.x;


        plot( ...
            trajectory(:,1), ...
            trajectory(:,3), ...
            'k-', ...
            'LineWidth', ...
            0.25);

    end


    xlabel('Cart position p [m]');
    ylabel('Pole angle \theta [rad]');


    title(sprintf( ...
        'Cart-Pole Reachability - %s - p-\\theta Projection', ...
        difficulty));


    pThetaFig = fullfile( ...
        outputFolder, ...
        sprintf( ...
            'CARTPOLE_REACH_%s_p_theta.fig', ...
            label));


    pThetaPNG = fullfile( ...
        outputFolder, ...
        sprintf( ...
            'CARTPOLE_REACH_%s_p_theta.png', ...
            label));


    savefig(pThetaFig);

    exportgraphics( ...
        gcf, ...
        pThetaPNG, ...
        'Resolution', ...
        300);


    close;


    %% ========================================================
    % 15. Plot projection: pole angle vs angular velocity
    % =========================================================

    figure;

    hold on;
    box on;
    grid on;


    plot(R,[3 4]);

    plot(params.R0,[3 4]);


    for j = 1:numberValidationTrajectories

        trajectory = ...
            validationTrajectories{j}.x;


        plot( ...
            trajectory(:,3), ...
            trajectory(:,4), ...
            'k-', ...
            'LineWidth', ...
            0.25);

    end


    xlabel('Pole angle \theta [rad]');
    ylabel('Angular velocity \omega [rad/s]');


    title(sprintf( ...
        'Cart-Pole Reachability - %s - \\theta-\\omega Projection', ...
        difficulty));


    thetaOmegaFig = fullfile( ...
        outputFolder, ...
        sprintf( ...
            'CARTPOLE_REACH_%s_theta_omega.fig', ...
            label));


    thetaOmegaPNG = fullfile( ...
        outputFolder, ...
        sprintf( ...
            'CARTPOLE_REACH_%s_theta_omega.png', ...
            label));


    savefig(thetaOmegaFig);

    exportgraphics( ...
        gcf, ...
        thetaOmegaPNG, ...
        'Resolution', ...
        300);


    close;


    %% ========================================================
    % 16. Plot projection: cart position vs cart velocity
    % =========================================================

    figure;

    hold on;
    box on;
    grid on;


    plot(R,[1 2]);

    plot(params.R0,[1 2]);


    for j = 1:numberValidationTrajectories

        trajectory = ...
            validationTrajectories{j}.x;


        plot( ...
            trajectory(:,1), ...
            trajectory(:,2), ...
            'k-', ...
            'LineWidth', ...
            0.25);

    end


    xlabel('Cart position p [m]');
    ylabel('Cart velocity v [m/s]');


    title(sprintf( ...
        'Cart-Pole Reachability - %s - p-v Projection', ...
        difficulty));


    pvFig = fullfile( ...
        outputFolder, ...
        sprintf( ...
            'CARTPOLE_REACH_%s_p_v.fig', ...
            label));


    pvPNG = fullfile( ...
        outputFolder, ...
        sprintf( ...
            'CARTPOLE_REACH_%s_p_v.png', ...
            label));


    savefig(pvFig);

    exportgraphics( ...
        gcf, ...
        pvPNG, ...
        'Resolution', ...
        300);


    close;


    %% ========================================================
    % 17. Store summary
    % =========================================================

    summaryDifficulty(k) = ...
        string(difficulty);

    summaryComputationTime(k) = ...
        computationTime;

    summaryInitialThetaDeg(k) = ...
        rad2deg(ub(3));

    summaryValidationCount(k) = ...
        numberValidationTrajectories;


    fprintf('\n%s reference completed.\n', ...
        difficulty);

end


%% ============================================================
% 18. Reference summary
% =============================================================

summaryTable = table( ...
    summaryDifficulty, ...
    summaryInitialThetaDeg, ...
    summaryComputationTime, ...
    summaryValidationCount, ...
    'VariableNames', { ...
        'Difficulty', ...
        'Initial_Theta_HalfWidth_deg', ...
        'CORA_Computation_Time_s', ...
        'Validation_Trajectories'});


fprintf('\n');
fprintf('=============================================\n');
fprintf('REFERENCE SUMMARY\n');
fprintf('=============================================\n');

disp(summaryTable);


summaryCSV = fullfile( ...
    outputFolder, ...
    'CARTPOLE_REACH_reference_summary.csv');


writetable( ...
    summaryTable, ...
    summaryCSV);


save( ...
    fullfile( ...
        outputFolder, ...
        'CARTPOLE_REACH_reference_summary.mat'), ...
    'summaryTable');


%% ============================================================
% 19. Finished
% =============================================================

fprintf('\n');
fprintf('=============================================\n');
fprintf('All Cart-Pole reachability references completed.\n');
fprintf('=============================================\n');


fprintf('\nGenerated reference files:\n');

fprintf('CARTPOLE_REACH_B_reference.mat\n');
fprintf('CARTPOLE_REACH_I_reference.mat\n');
fprintf('CARTPOLE_REACH_C_reference.mat\n');


fprintf('\nGenerated validation files:\n');

fprintf('CARTPOLE_REACH_B_validation.mat\n');
fprintf('CARTPOLE_REACH_I_validation.mat\n');
fprintf('CARTPOLE_REACH_C_validation.mat\n');


fprintf('\nSummary file:\n');

fprintf('CARTPOLE_REACH_reference_summary.csv\n');


fprintf('\nOutput folder:\n');

disp(fullfile(pwd,outputFolder));


%% ============================================================
% LOCAL FUNCTION
% Nonlinear Cart-Pole dynamics
% =============================================================

function dx = localCartPoleDynamics( ...
    x, ...
    u, ...
    M, ...
    m, ...
    l, ...
    g)


    % State extraction
    v = x(2);

    theta = x(3);

    omega = x(4);


    % Input
    force = u(1);


    sinTheta = sin(theta);
    cosTheta = cos(theta);


    denominator = ...
        M + ...
        m*sinTheta^2;


    % Cart acceleration
    vDot = ...
        ( ...
        force ...
        - m*g*sinTheta*cosTheta ...
        + m*l*omega^2*sinTheta ...
        ) ...
        / denominator;


    % Pole angular acceleration
    omegaDot = ...
        ( ...
        g*sinTheta ...
        - vDot*cosTheta ...
        ) ...
        / l;


    % Complete derivative
    dx = [ ...
        v; ...
        vDot; ...
        omega; ...
        omegaDot];

end