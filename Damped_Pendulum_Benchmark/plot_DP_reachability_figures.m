%% Figure 1: Three Damped Pendulum Reachability References

clear;
clc;
close all;

files = { ...
    'DP_REACH_B_reference.mat', ...
    'DP_REACH_I_reference.mat', ...
    'DP_REACH_C_reference.mat'};

titles = { ...
    'Baseline', ...
    'Intermediate', ...
    'Challenge'};

figure('Position',[100 100 1200 400]);

tiledlayout(1,3,'TileSpacing','compact','Padding','compact');

for k = 1:3

    load(files{k},'R','params');

    nexttile;
    hold on;
    box on;

    % CORA reachable flowpipe
    plot(R,[1 2]);

    % Initial set
    plot(params.R0,[1 2]);

    xlabel('\theta [rad]');
    ylabel('\omega [rad/s]');
    title(titles{k});

    grid on;
end

sgtitle('Damped Pendulum Reachability References, T = 2.0 s');

% Save vector PDF
exportgraphics(gcf, ...
    'DP_reachability_three_levels.pdf', ...
    'ContentType','vector');

% Save high-resolution PNG
exportgraphics(gcf, ...
    'DP_reachability_three_levels.png', ...
    'Resolution',300);
%% Figure 2: Challenge Flowpipe with Independent Trajectory Samples

clear R params metadata;

load('DP_REACH_C_reference.mat', ...
    'R','params','metadata');

% Damped Pendulum dynamics
odefun = @(t,x) [ ...
    x(2); ...
    -0.2*x(2) - 9.81*sin(x(1)) ...
];

% High-accuracy ODE settings
odeOptions = odeset( ...
    'RelTol',1e-12, ...
    'AbsTol',1e-14);

% Challenge initial set
lb = metadata.initialLowerBound;
ub = metadata.initialUpperBound;

% Fixed random seed for reproducibility
rng(2026);

% Number of sampled trajectories
N = 100;

Xsample = lb + (ub-lb).*rand(2,N);

figure('Position',[100 100 800 600]);
hold on;
box on;

% Plot CORA reference flowpipe
plot(R,[1 2]);

% Plot initial set
plot(params.R0,[1 2]);

% Independently simulated trajectories
for j = 1:N

    x0 = Xsample(:,j);

    [tSim,xSim] = ode113( ...
        odefun, ...
        [0 2.0], ...
        x0, ...
        odeOptions);

    plot(xSim(:,1),xSim(:,2), ...
        'k-', ...
        'LineWidth',0.4);
end

xlabel('\theta [rad]');
ylabel('\omega [rad/s]');

title(['Challenge Reachability Reference with ', ...
       'High-Accuracy Sampled Trajectories']);

grid on;

exportgraphics(gcf, ...
    'DP_REACH_C_validation.pdf', ...
    'ContentType','vector');

exportgraphics(gcf, ...
    'DP_REACH_C_validation.png', ...
    'Resolution',300);