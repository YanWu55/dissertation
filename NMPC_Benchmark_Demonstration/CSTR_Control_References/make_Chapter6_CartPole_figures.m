clear;
clc;
close all;

%% ============================================================
% Chapter 6 Cart-Pole figures
% Uses the REVISED shared CP-1 benchmark results
% =============================================================

summaryFile = 'CARTPOLE_CTRL_reference_summary.csv';
challengeFile = 'CARTPOLE_CTRL_C_reference.mat';

%% ============================================================
% Figure 6.2
% Settling-time comparison
% =============================================================

T = readtable( ...
    'CARTPOLE_CTRL_reference_summary.csv', ...
    'TextType','string');

lqrSettling  = T.LQR_Settling_Time_s;
nmpcSettling = T.NMPC_Settling_Time_s;

%% ============================================================
% Unified font settings
% =============================================================

fontAxis   = 17;
fontLabel  = 17;
fontLegend = 17;
fontAnnot  = 17;
fontTitle  = 17;

%% ============================================================
% Create figure
% =============================================================

figure('Color','w', ...
       'Position',[100 100 1050 600]);

Y = [lqrSettling, nmpcSettling];

b = bar(Y,'grouped');

grid on;
box on;

ylabel('Settling time [s]', ...
    'FontName','Times New Roman', ...
    'FontSize',fontLabel);

%% ------------------------------------------------------------
% X-axis
% ------------------------------------------------------------

xticks(1:3);

% Do not use multiline xticklabels directly
xticklabels({});

xlim([0.35 3.65]);

% Custom two-line labels
caseNames = {'Baseline', 'Intermediate', 'Challenge'};
angles    = {'(10°)', '(30°)', '(40°)'};

yName  = -0.18;
yAngle = -0.42;

for i = 1:3

    text(i, yName, caseNames{i}, ...
        'HorizontalAlignment','center', ...
        'VerticalAlignment','top', ...
        'FontName','Times New Roman', ...
        'FontSize',fontAxis, ...
        'Interpreter','none', ...
        'Clipping','off');

    text(i, yAngle, angles{i}, ...
        'HorizontalAlignment','center', ...
        'VerticalAlignment','top', ...
        'FontName','Times New Roman', ...
        'FontSize',fontAxis, ...
        'Interpreter','none', ...
        'Clipping','off');

end

%% ------------------------------------------------------------
% Legend
% ------------------------------------------------------------

lgd = legend( ...
    'Reconstructed LQR', ...
    'NMPC', ...
    'Location','northwest');

set(lgd, ...
    'FontName','Times New Roman', ...
    'FontSize',fontLegend);

%% ------------------------------------------------------------
% Axis formatting
% ------------------------------------------------------------

ylim([0 6.4]);

ax = gca;

set(ax, ...
    'FontName','Times New Roman', ...
    'FontSize',fontAxis, ...
    'LineWidth',1.2);

% More space at the bottom for two-line labels
ax.Position = [0.10 0.22 0.84 0.70];

%% ------------------------------------------------------------
% X-axis title
% ------------------------------------------------------------

xlabel('Benchmark instance', ...
    'FontName','Times New Roman', ...
    'FontSize',fontLabel);

% Move xlabel further downward
ax.XLabel.Units = 'normalized';
ax.XLabel.Position = [0.5 -0.16 0];

%% ------------------------------------------------------------
% Numerical labels above bars
% ------------------------------------------------------------

for k = 1:numel(b)

    xEnd = b(k).XEndPoints;
    yEnd = b(k).YEndPoints;

    labels = string(round(b(k).YData,1));

    text( ...
        xEnd, ...
        yEnd, ...
        labels, ...
        'HorizontalAlignment','center', ...
        'VerticalAlignment','bottom', ...
        'FontName','Times New Roman', ...
        'FontSize',fontAnnot);

end

%% ------------------------------------------------------------
% Title
% ------------------------------------------------------------

title( ...
    'Settling-Time Comparison of Reconstructed LQR and NMPC', ...
    'FontName','Times New Roman', ...
    'FontSize',fontTitle, ...
    'FontWeight','normal');

%% ------------------------------------------------------------
% Export
% ------------------------------------------------------------

exportgraphics( ...
    gcf, ...
    'Figure_6_2_CartPole_settling_time.png', ...
    'Resolution',600);

exportgraphics( ...
    gcf, ...
    'Figure_6_2_CartPole_settling_time.pdf', ...
    'ContentType','vector');

fprintf('\nFigure 6.2 generated successfully.\n');
%% ============================================================
% Figure 6.3
% Representative 40-degree Challenge responses
% =============================================================

S = load('CARTPOLE_CTRL_C_reference.mat');

lqr  = S.lqrResult;
nmpc = S.nmpcResult;

% ------------------------------------------------------------
% Stored state order:
%
% x(1,:) = cart position z
% x(2,:) = pole angle theta
% x(3,:) = cart velocity z_dot
% x(4,:) = pole angular velocity theta_dot
% ------------------------------------------------------------

tLQR  = lqr.t(:);
tNMPC = nmpc.t(:);

thetaLQR  = rad2deg(lqr.x(2,:)).';
thetaNMPC = rad2deg(nmpc.x(2,:)).';

zLQR  = lqr.x(1,:).';
zNMPC = nmpc.x(1,:).';

tuLQR  = lqr.tu(:);
tuNMPC = nmpc.tu(:);

uLQR  = lqr.u(:);
uNMPC = nmpc.u(:);

% Revised shared CP-1 constraints
zMax = 2.4;     % m
uMax = 100;     % N

% Settling information
lqrSettling  = lqr.metrics.SettlingTime;
nmpcSettling = nmpc.metrics.SettlingTime;

lqrMaxCart  = lqr.metrics.MaxCartPosition;
nmpcMaxCart = nmpc.metrics.MaxCartPosition;

lqrPeakU  = lqr.metrics.PeakInput;
nmpcPeakU = nmpc.metrics.PeakInput;

%% ============================================================
% Unified font settings
% =============================================================

fontAxis   = 17;
fontLabel  = 17;
fontLegend = 17;
fontAnnot  = 17;
fontTitle  = 17;

%% ============================================================
% Create figure
% =============================================================

figure('Color','w', ...
       'Position',[100 40 980 1080]);

tl = tiledlayout(3,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');

%% ------------------------------------------------------------
% (a) Pole angle
% ------------------------------------------------------------

ax1 = nexttile;

plot(tLQR,thetaLQR, ...
    'LineWidth',1.8);

hold on;

plot(tNMPC,thetaNMPC, ...
    'LineWidth',1.8);

yline(0,'--', ...
    'LineWidth',1.2);

grid on;
box on;

xlim([0 8]);

ylabel('Pole angle, \theta [deg]', ...
    'FontName','Times New Roman', ...
    'FontSize',fontLabel);

lgd1 = legend( ...
    'Reconstructed LQR', ...
    'NMPC', ...
    'Upright target', ...
    'Location','northeast');

set(lgd1, ...
    'FontName','Times New Roman', ...
    'FontSize',fontLegend);

set(ax1, ...
    'FontName','Times New Roman', ...
    'FontSize',fontAxis, ...
    'LineWidth',1.2);

text(0.15,0.08, ...
    sprintf(['Settling time: LQR = %.1f s, ' ...
             'NMPC = %.1f s'], ...
             lqrSettling,nmpcSettling), ...
    'Units','normalized', ...
    'FontName','Times New Roman', ...
    'FontSize',17, ...
    'BackgroundColor','w');

%% ------------------------------------------------------------
% (b) Cart position
% ------------------------------------------------------------

ax2 = nexttile;

plot(tLQR,zLQR, ...
    'LineWidth',1.8);

hold on;

plot(tNMPC,zNMPC, ...
    'LineWidth',1.8);

yline(zMax,'--', ...
    'LineWidth',1.2);

yline(-zMax,'--', ...
    'LineWidth',1.2);

grid on;
box on;

xlim([0 8]);
ylim([-2.6 2.6]);

ylabel('Cart position, z [m]', ...
    'FontName','Times New Roman', ...
    'FontSize',fontLabel);

lgd2 = legend( ...
    'Reconstructed LQR', ...
    'NMPC', ...
    'Track constraint', ...
    'Location','northeast');

set(lgd2, ...
    'FontName','Times New Roman', ...
    'FontSize',fontLegend);

set(ax2, ...
    'FontName','Times New Roman', ...
    'FontSize',fontAxis, ...
    'LineWidth',1.2);

text(0.02,0.08, ...
    sprintf(['Peak |z|: LQR = %.3f m, ' ...
             'NMPC = %.3f m'], ...
             lqrMaxCart,nmpcMaxCart), ...
    'Units','normalized', ...
    'FontName','Times New Roman', ...
    'FontSize',fontAnnot, ...
    'BackgroundColor','w');

%% ------------------------------------------------------------
% (c) Control input
% ------------------------------------------------------------

ax3 = nexttile;

stairs(tuLQR,uLQR, ...
    'LineWidth',1.6);

hold on;

stairs(tuNMPC,uNMPC, ...
    'LineWidth',1.6);

yline(uMax,'--', ...
    'LineWidth',1.2);

yline(-uMax,'--', ...
    'LineWidth',1.2);

grid on;
box on;

xlim([0 8]);
ylim([-110 110]);

xlabel('Time [s]', ...
    'FontName','Times New Roman', ...
    'FontSize',fontLabel);

ylabel('Control input, u [N]', ...
    'FontName','Times New Roman', ...
    'FontSize',fontLabel);

lgd3 = legend( ...
    'Reconstructed LQR', ...
    'NMPC', ...
    'Input constraint', ...
    'Location','northeast');

set(lgd3, ...
    'FontName','Times New Roman', ...
    'FontSize',fontLegend);

set(ax3, ...
    'FontName','Times New Roman', ...
    'FontSize',fontAxis, ...
    'LineWidth',1.2);

text(0.02,0.08, ...
    sprintf(['Peak |u|: LQR = %.2f N, ' ...
             'NMPC = %.2f N'], ...
             lqrPeakU,nmpcPeakU), ...
    'Units','normalized', ...
    'FontName','Times New Roman', ...
    'FontSize',fontAnnot, ...
    'BackgroundColor','w');

%% ============================================================
% Overall figure title
% =============================================================

title(tl, ...
    'Representative 40° Challenge Cart-Pole Responses', ...
    'FontName','Times New Roman', ...
    'FontSize',fontTitle, ...
    'FontWeight','normal');

%% ============================================================
% Export
% =============================================================

exportgraphics(gcf, ...
    'Figure_6_3_CartPole_40deg_responses.png', ...
    'Resolution',600);

exportgraphics(gcf, ...
    'Figure_6_3_CartPole_40deg_responses.pdf', ...
    'ContentType','vector');

fprintf('\nFigure 6.3 generated successfully.\n');