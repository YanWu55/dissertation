function summaryTable = run_CartPole_reachability_references_CP1()
%RUN_CARTPOLE_REACHABILITY_REFERENCES_CP1
% A.6.3 Cart-Pole Open-Loop Unstable Reachability
%
% Shared plant: the ACTUAL CP-1 model from cartpole_params.m /
% cartpole_state_ct.m.
%
% State order:
%   x = [z; theta; z_dot; theta_dot]
%
% Physical parameters are loaded directly from cartpole_params.m:
%   M = 1 kg
%   m = 1 kg
%   l = 0.5 m
%   g = 9.81 m/s^2
%   Kd = 10 N s/m
%
% Reachability family:
%   u(t) = 0 exactly
%   horizon = 1.0 s for B / I / C
%
% Initial sets:
%   B: z +/-0.01 m, theta +/-1 deg, z_dot +/-0.01 m/s,
%      theta_dot +/-0.01 rad/s
%   I: z +/-0.05 m, theta +/-3 deg, z_dot +/-0.05 m/s,
%      theta_dot +/-0.05 rad/s
%   C: z +/-0.10 m, theta +/-5 deg, z_dot +/-0.10 m/s,
%      theta_dot +/-0.10 rad/s
%
% Primary reference:
%   CORA nonlinear set-based reachability, poly-adaptive
%
% Independent consistency check:
%   100 uniformly sampled initial states per case, ode113
%
% Output files:
%   CARTPOLE_REACH_B_reference.mat
%   CARTPOLE_REACH_I_reference.mat
%   CARTPOLE_REACH_C_reference.mat
%   CARTPOLE_REACH_B_validation.mat
%   CARTPOLE_REACH_I_validation.mat
%   CARTPOLE_REACH_C_validation.mat
%   CARTPOLE_REACH_reference_summary.csv
%
% NOTE:
%   Set makeFigures = true only if you want the projection figures.
%   Keeping it false makes the expensive CORA reference run finish/save
%   before any plotting work.

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

if isempty(which('nonlinearSys'))
    error(['CORA cannot be found on the MATLAB path. Add the CORA root ', ...
        'directory before running this script.']);
end

fprintf('MATLAB version: %s\n', version);
fprintf('Computer platform: %s\n', computer);

try
    fprintf('CORA version: %s\n', CORAVERSION);
catch
    warning('CORA version could not be read automatically.');
end

%% ------------------------------------------------------------------------
% 1. Load the actual shared CP-1 plant
% -------------------------------------------------------------------------

p = cartpole_params();

fprintf('\n============================================================\n');
fprintf('A.6.3 Cart-Pole Reachability — shared CP-1 plant\n');
fprintf('============================================================\n');
fprintf('State order: [z theta z_dot theta_dot]\n');
fprintf('M  = %.6f kg\n', p.M);
fprintf('m  = %.6f kg\n', p.m);
fprintf('l  = %.6f m\n', p.l);
fprintf('g  = %.6f m/s^2\n', p.g);
fprintf('Kd = %.6f N s/m\n', p.Kd);
fprintf('CP-1 track constraint: [%.3f, %.3f] m\n', p.zMin, p.zMax);
fprintf('CP-1 force constraint: [%.3f, %.3f] N\n', p.uMin, p.uMax);

Tfinal = 1.0;
fprintf('Reachability horizon: %.3f s\n', Tfinal);
fprintf('Reachability input: u(t) = 0 N\n');

outputFolder = 'CartPole_Reachability_References';
if ~exist(outputFolder,'dir')
    mkdir(outputFolder);
end

makeFigures = false;

%% ------------------------------------------------------------------------
% 2. CORA-compatible nonlinear dynamics
%
% This is a direct transcription of cartpole_state_ct.m:
%
%   x(1) = z
%   x(2) = theta
%   x(3) = z_dot
%   x(4) = theta_dot
% -------------------------------------------------------------------------

f = @(x,u) [ ...
    x(3); ...
    x(4); ...
    ( ...
        u(1) ...
        - p.Kd*x(3) ...
        - p.m*p.l*x(4)^2*sin(x(2)) ...
        + p.m*p.g*sin(x(2))*cos(x(2)) ...
    ) / (p.M + p.m*sin(x(2))^2); ...
    ( ...
        p.g*sin(x(2)) ...
        + ( ...
            u(1) ...
            - p.Kd*x(3) ...
            - p.m*p.l*x(4)^2*sin(x(2)) ...
          ) * cos(x(2)) / (p.M + p.m) ...
    ) / ( ...
        p.l ...
        - p.m*p.l*cos(x(2))^2/(p.M+p.m) ...
    ) ...
    ];

sys = nonlinearSys('CartPole_CP1_OpenLoop',f,4,1);

%% ------------------------------------------------------------------------
% 3. Dynamics consistency checks against the supplied CP-1 implementation
% -------------------------------------------------------------------------

upright = [0;0;0;0];

dxLocal = f(upright,0);
dxProject = cartpole_state_ct(upright,0);

fprintf('\nDynamics at upright equilibrium:\n');
disp(dxLocal);

if norm(dxLocal,inf) > 1e-12
    error('The local reachability dynamics do not preserve the upright equilibrium.');
end

if norm(dxLocal-dxProject,inf) > 1e-12
    error('Local CORA dynamics do not match cartpole_state_ct at the equilibrium.');
end

testStates = [ ...
     0.12, deg2rad( 4),  0.08, -0.12; ...
    -0.20, deg2rad(-7), -0.15,  0.20; ...
     0.35, deg2rad(10),  0.25, -0.30]';

for j = 1:size(testStates,2)
    xTest = testStates(:,j);
    d1 = f(xTest,0);
    d2 = cartpole_state_ct(xTest,0);

    if norm(d1-d2,inf) > 1e-11
        error('Local CORA dynamics mismatch cartpole_state_ct at test state %d.',j);
    end
end

fprintf('CP-1 dynamics consistency checks passed.\n');

%% ------------------------------------------------------------------------
% 4. Common CORA configuration
% -------------------------------------------------------------------------

params.tStart = 0;
params.tFinal = Tfinal;

% Exact zero force.
params.U = zonotope(0,0);

options.alg = 'poly-adaptive';

%% ------------------------------------------------------------------------
% 5. Difficulty levels — same physical boxes as the previous A.6.3,
%    reordered to the actual CP-1 state order.
% -------------------------------------------------------------------------

cases(1).label = 'B';
cases(1).difficulty = 'Baseline';
cases(1).lb = [ ...
    -0.01; ...
    -deg2rad(1); ...
    -0.01; ...
    -0.01];
cases(1).ub = [ ...
     0.01; ...
     deg2rad(1); ...
     0.01; ...
     0.01];

cases(2).label = 'I';
cases(2).difficulty = 'Intermediate';
cases(2).lb = [ ...
    -0.05; ...
    -deg2rad(3); ...
    -0.05; ...
    -0.05];
cases(2).ub = [ ...
     0.05; ...
     deg2rad(3); ...
     0.05; ...
     0.05];

cases(3).label = 'C';
cases(3).difficulty = 'Challenge';
cases(3).lb = [ ...
    -0.10; ...
    -deg2rad(5); ...
    -0.10; ...
    -0.10];
cases(3).ub = [ ...
     0.10; ...
     deg2rad(5); ...
     0.10; ...
     0.10];

%% ------------------------------------------------------------------------
% 6. Independent trajectory validation
% -------------------------------------------------------------------------

numberValidationTrajectories = 100;

rng(2026,'twister');

validationRelTol = 1e-12;
validationAbsTol = 1e-14;
validationMaxStep = 1e-3;
validationOutputStep = 1e-3;

tEval = (0:validationOutputStep:Tfinal)';

odeOptions = odeset( ...
    'RelTol',validationRelTol, ...
    'AbsTol',validationAbsTol, ...
    'MaxStep',validationMaxStep);

%% ------------------------------------------------------------------------
% 7. Summary storage
% -------------------------------------------------------------------------

Difficulty = strings(3,1);
Initial_Theta_HalfWidth_deg = zeros(3,1);
CORA_Computation_Time_s = zeros(3,1);
Validation_Trajectories = zeros(3,1);

%% ------------------------------------------------------------------------
% 8. Run B / I / C
% -------------------------------------------------------------------------

for k = 1:numel(cases)

    label = cases(k).label;
    difficulty = cases(k).difficulty;
    lb = cases(k).lb;
    ub = cases(k).ub;

    fprintf('\n============================================================\n');
    fprintf('Running %s reference (%s)\n',label,difficulty);
    fprintf('============================================================\n');
    fprintf('Initial lower bound [z theta z_dot theta_dot]:\n');
    disp(lb');
    fprintf('Initial upper bound [z theta z_dot theta_dot]:\n');
    disp(ub');

    X0interval = interval(lb,ub);
    params.R0 = polyZonotope(X0interval);

    %% CORA reference
    reachTimer = tic;

    try
        R = reach(sys,params,options);
    catch ME
        fprintf('\nERROR during %s CORA reachability.\n',difficulty);
        fprintf('%s\n',ME.message);
        rethrow(ME);
    end

    computationTime = toc(reachTimer);

    fprintf('CORA reachability completed successfully.\n');
    fprintf('Computation time: %.6f s\n',computationTime);

    %% Independent high-accuracy sampled trajectories
    fprintf('Generating %d independent validation trajectories...\n', ...
        numberValidationTrajectories);

    validationTrajectories = cell(numberValidationTrajectories,1);
    validationInitialStates = zeros(4,numberValidationTrajectories);

    odeFun = @(t,x) localPlantDynamics(x,0,p); %#ok<INUSD>

    for j = 1:numberValidationTrajectories
        x0 = lb + (ub-lb).*rand(4,1);
        validationInitialStates(:,j) = x0;

        [tTrajectory,xTrajectory] = ode113( ...
            odeFun,tEval,x0,odeOptions);

        validationTrajectories{j}.t = tTrajectory;
        validationTrajectories{j}.x = xTrajectory;
    end

    fprintf('Validation trajectories completed.\n');

    %% Metadata
    metadata.system = 'Cart-Pole';
    metadata.task = 'Reachability';
    metadata.benchmarkFamily = 'Open-Loop Unstable Reachability';
    metadata.sharedPlant = 'CP-1 actual benchmark configuration';
    metadata.difficulty = difficulty;
    metadata.stateOrder = 'z theta z_dot theta_dot';
    metadata.angleConvention = 'theta = 0 is upright';

    metadata.cartMass_kg = p.M;
    metadata.poleMass_kg = p.m;
    metadata.poleLength_m = p.l;
    metadata.gravity_mps2 = p.g;
    metadata.cartDamping_Ns_per_m = p.Kd;

    metadata.cp1CartPositionMin_m = p.zMin;
    metadata.cp1CartPositionMax_m = p.zMax;
    metadata.cp1ForceMin_N = p.uMin;
    metadata.cp1ForceMax_N = p.uMax;

    metadata.inputForce_N = 0;
    metadata.initialLowerBound = lb;
    metadata.initialUpperBound = ub;
    metadata.tStart = 0;
    metadata.tFinal = Tfinal;

    metadata.algorithm = 'poly-adaptive';
    metadata.initialSetRepresentation = 'Polynomial zonotope';
    metadata.inputSetRepresentation = 'Exact zero-input zonotope';
    metadata.referenceType = ...
        'Numerical - set-based reachability reference';

    metadata.computationTime_seconds = computationTime;

    metadata.validationTrajectoryCount = numberValidationTrajectories;
    metadata.validationSolver = 'MATLAB ode113';
    metadata.validationRelTol = validationRelTol;
    metadata.validationAbsTol = validationAbsTol;
    metadata.validationMaxStep_seconds = validationMaxStep;
    metadata.validationOutputStep_seconds = validationOutputStep;
    metadata.randomSeed = 2026;

    metadata.matlabVersion = version;
    metadata.computer = computer;

    try
        metadata.coraVersion = CORAVERSION;
    catch
        metadata.coraVersion = 'CORA version not read automatically';
    end

    metadata.dateGenerated = char(datetime('now'));

    %% Save primary reference
    referenceFile = fullfile(outputFolder, ...
        sprintf('CARTPOLE_REACH_%s_reference.mat',label));

    save(referenceFile, ...
        'R','sys','params','options','metadata','p','lb','ub','-v7.3');

    fprintf('Saved reference file:\n%s\n',referenceFile);

    %% Save validation data
    validationMetadata.description = [ ...
        'Independent high-accuracy sampled trajectories used as a ', ...
        'numerical consistency check. These trajectories do not replace ', ...
        'the set-based CORA reference.'];
    validationMetadata.numberTrajectories = numberValidationTrajectories;
    validationMetadata.solver = 'ode113';
    validationMetadata.RelTol = validationRelTol;
    validationMetadata.AbsTol = validationAbsTol;
    validationMetadata.MaxStep_seconds = validationMaxStep;
    validationMetadata.outputStep_seconds = validationOutputStep;
    validationMetadata.randomSeed = 2026;
    validationMetadata.stateOrder = 'z theta z_dot theta_dot';

    validationFile = fullfile(outputFolder, ...
        sprintf('CARTPOLE_REACH_%s_validation.mat',label));

    save(validationFile, ...
        'validationTrajectories','validationInitialStates', ...
        'validationMetadata','tEval','-v7.3');

    fprintf('Saved validation file:\n%s\n',validationFile);

    %% Optional figures
    if makeFigures
        localMakeReachabilityFigures( ...
            R,params.R0,validationTrajectories,difficulty,label,outputFolder);
    end

    %% Summary
    Difficulty(k) = string(difficulty);
    Initial_Theta_HalfWidth_deg(k) = rad2deg(ub(2));
    CORA_Computation_Time_s(k) = computationTime;
    Validation_Trajectories(k) = numberValidationTrajectories;

    fprintf('%s reference completed.\n',difficulty);
end

%% ------------------------------------------------------------------------
% 9. Summary
% -------------------------------------------------------------------------

summaryTable = table( ...
    Difficulty, ...
    Initial_Theta_HalfWidth_deg, ...
    CORA_Computation_Time_s, ...
    Validation_Trajectories);

fprintf('\n============================================================\n');
fprintf('REFERENCE SUMMARY\n');
fprintf('============================================================\n');
disp(summaryTable);

writetable(summaryTable, ...
    fullfile(outputFolder,'CARTPOLE_REACH_reference_summary.csv'));

save(fullfile(outputFolder,'CARTPOLE_REACH_reference_summary.mat'), ...
    'summaryTable','p');

fprintf('\n============================================================\n');
fprintf('All shared-plant CP-1 reachability references completed.\n');
fprintf('============================================================\n');
fprintf('Output folder:\n%s\n',fullfile(pwd,outputFolder));

end


%% =========================================================================
% LOCAL FUNCTION: exact CP-1 continuous-time dynamics
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
% LOCAL FUNCTION: optional figures
% =========================================================================

function localMakeReachabilityFigures( ...
    R,R0,validationTrajectories,difficulty,label,outputFolder)

projections = { ...
    [1 2], 'z_theta', 'Cart position z [m]', 'Pole angle \theta [rad]'; ...
    [2 4], 'theta_thetadot', 'Pole angle \theta [rad]', ...
        'Pole angular velocity \theta_dot [rad/s]'; ...
    [1 3], 'z_zdot', 'Cart position z [m]', 'Cart velocity z_dot [m/s]'};

for q = 1:size(projections,1)
    dims = projections{q,1};
    suffix = projections{q,2};
    xLabelText = projections{q,3};
    yLabelText = projections{q,4};

    figure;
    hold on;
    box on;
    grid on;

    plot(R,dims);
    plot(R0,dims);

    for j = 1:numel(validationTrajectories)
        X = validationTrajectories{j}.x;
        plot(X(:,dims(1)),X(:,dims(2)),'k-','LineWidth',0.25);
    end

    xlabel(xLabelText);
    ylabel(yLabelText);
    title(sprintf('Cart-Pole Reachability - %s',difficulty));

    exportgraphics(gcf, ...
        fullfile(outputFolder, ...
        sprintf('CARTPOLE_REACH_%s_%s.png',label,suffix)), ...
        'Resolution',300);

    savefig(fullfile(outputFolder, ...
        sprintf('CARTPOLE_REACH_%s_%s.fig',label,suffix)));

    close;
end

end
