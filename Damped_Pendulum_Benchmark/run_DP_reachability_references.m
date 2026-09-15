%% Damped Pendulum Reachability Reference Generation
% CORA v2026.1.0
%
% Generates:
%   DP_REACH_B_reference.mat
%   DP_REACH_I_reference.mat
%   DP_REACH_C_reference.mat
%
% Benchmark model:
%   theta_dot = omega
%   omega_dot = -(b/(m*l^2))*omega - (g/l)*sin(theta)
%
% Benchmark horizon:
%   T = 2.0 s

clear;
clc;
close all;

%% ------------------------------------------------------------
% 0. Check CORA installation
% -------------------------------------------------------------

if isempty(which('nonlinearSys'))
    error(['CORA cannot be found on the MATLAB path. ' ...
           'Add the CORA root directory using addpath(genpath(...)).']);
end

fprintf('MATLAB version: %s\n', version);

try
    fprintf('CORA version: %s\n', CORAVERSION);
catch
    warning('CORA version could not be read automatically.');
end


%% ------------------------------------------------------------
% 1. Damped Pendulum Parameters
% -------------------------------------------------------------

m = 1.0;       % kg
l = 1.0;       % m
b = 0.2;       % N m s / rad
g = 9.81;      % m / s^2

Tfinal = 2.0;  % benchmark horizon [s]


%% ------------------------------------------------------------
% 2. Nonlinear Dynamics
%
% State:
%   x(1) = theta [rad]
%   x(2) = omega [rad/s]
%
% A dummy zero input is included because CORA nonlinearSys
% uses the general form xdot = f(x,u).
% -------------------------------------------------------------

f = @(x,u) [x(2); -0.2*x(2) - 9.81*sin(x(1));

sys = nonlinearSys(f);


%% ------------------------------------------------------------
% 3. Fixed Reachability Configuration
% -------------------------------------------------------------

params.tStart = 0;
params.tFinal = Tfinal;



% CORA adaptive nonlinear reachability
options.alg = 'poly-adaptive';


%% ------------------------------------------------------------
% 4. Define the three benchmark instances
% -------------------------------------------------------------

cases(1).label = 'B';
cases(1).difficulty = 'Baseline';
cases(1).lb = [-0.05; -0.05];
cases(1).ub = [ 0.05;  0.05];

cases(2).label = 'I';
cases(2).difficulty = 'Intermediate';
cases(2).lb = [-0.25; -0.25];
cases(2).ub = [ 0.25;  0.25];

cases(3).label = 'C';
cases(3).difficulty = 'Challenge';
cases(3).lb = [-1.00; -0.50];
cases(3).ub = [ 1.00;  0.50];


%% ------------------------------------------------------------
% 5. Run all three reference computations
% -------------------------------------------------------------

for k = 1:length(cases)

    fprintf('\n');
    fprintf('=============================================\n');
    fprintf('Running %s reference (%s)\n', ...
        cases(k).label, cases(k).difficulty);
    fprintf('=============================================\n');

    lb = cases(k).lb;
    ub = cases(k).ub;

    % Initial interval box
    X0_interval = interval(lb, ub);

    % Convert interval to polynomial zonotope
    params.R0 = polyZonotope(X0_interval);

    % ---------------------------------------------------------
    % Reachability computation
    % ----------------------------------------------------------

    tic;
    R = reach(sys, params, options);
    computationTime = toc;

    fprintf('Completed in %.6f s\n', computationTime);


    %% --------------------------------------------------------
    % 6. Store reproducibility metadata
    % ---------------------------------------------------------

    metadata.system = 'Damped Pendulum';
    metadata.task = 'Finite-Horizon Reachability';
    metadata.difficulty = cases(k).difficulty;

    metadata.mass = m;
    metadata.length = l;
    metadata.damping = b;
    metadata.gravity = g;

    metadata.initialLowerBound = lb;
    metadata.initialUpperBound = ub;

    metadata.tStart = 0;
    metadata.tFinal = Tfinal;

    metadata.algorithm = 'poly-adaptive';
    metadata.setRepresentation = 'Polynomial zonotope';

    metadata.computationTime_seconds = computationTime;

    metadata.matlabVersion = version;

    try
        metadata.coraVersion = CORAVERSION;
    catch
        metadata.coraVersion = 'v2026.1.0';
    end

    metadata.dateGenerated = char(datetime('now'));

    metadata.referenceType = ...
        'Numerical - validated set-based reachability reference';


    %% --------------------------------------------------------
    % 7. Save the complete CORA reference object
    % ---------------------------------------------------------

    fileName = sprintf( ...
        'DP_REACH_%s_reference.mat', cases(k).label);

    save(fileName, ...
        'R', ...
        'sys', ...
        'params', ...
        'options', ...
        'metadata', ...
        '-v7.3');

    fprintf('Saved reference file: %s\n', fileName);


    %% --------------------------------------------------------
    % 8. Plot the reference flowpipe
    % ---------------------------------------------------------

    figure;
    hold on;
    box on;

    plot(R, [1 2]);
    plot(params.R0, [1 2]);

    xlabel('\theta [rad]');
    ylabel('\omega [rad/s]');

    title(sprintf( ...
        'Damped Pendulum Reachability - %s', ...
        cases(k).difficulty));

    grid on;

    figName = sprintf( ...
        'DP_REACH_%s_flowpipe.fig', cases(k).label);

    savefig(figName);

    fprintf('Saved figure: %s\n', figName);

end


%% ------------------------------------------------------------
% 9. Finished
% -------------------------------------------------------------

fprintf('\n');
fprintf('=============================================\n');
fprintf('All three reference runs completed.\n');
fprintf('=============================================\n');

fprintf('\nGenerated files:\n');
fprintf('DP_REACH_B_reference.mat\n');
fprintf('DP_REACH_I_reference.mat\n');
fprintf('DP_REACH_C_reference.mat\n');