function make_Figure6_1_VanDerPol()
%MAKE_FIGURE6_1_VANDERPOL
%
% Chapter 6 — Figure 6.1
%
% Reference and computed Van der Pol periodic orbits
% for mu = 1, 5, and 10.
%
% Curves:
%   1. High-accuracy numerical reference
%   2. Long-time numerical integration
%   3. Shooting method
%
% Requires:
%   MATLAB
%   Optimization Toolbox (for fsolve)

clc;
close all;


%% ============================================================
% 1. Benchmark instances
% =============================================================

muValues = [1 5 10];

% Values currently reported in Chapter 6.
chapterReferencePeriod = [ ...
    6.663286859, ...
   11.612230668, ...
   19.078369567];

chapterReferenceAmplitude = [ ...
    2.008620, ...
    2.021508, ...
    2.014285];


%% ============================================================
% 2. Storage
% =============================================================

nCases = numel(muValues);

Tref   = zeros(1,nCases);
Tlong  = zeros(1,nCases);
Tshoot = zeros(1,nCases);

Aref   = zeros(1,nCases);
Along  = zeros(1,nCases);
Ashoot = zeros(1,nCases);

referenceOrbit = cell(1,nCases);
longOrbit      = cell(1,nCases);
shootOrbit     = cell(1,nCases);


%% ============================================================
% 3. Loop over mu = 1, 5, 10
% =============================================================

for i = 1:nCases

    mu = muValues(i);

    fprintf('\n');
    fprintf('====================================================\n');
    fprintf('Van der Pol: mu = %g\n',mu);
    fprintf('====================================================\n');


    %% --------------------------------------------------------
    % 3.1 Long-time integration
    % ---------------------------------------------------------

    x0 = [2;0];

    if mu == 1
        tFinal = 200;
    elseif mu == 5
        tFinal = 400;
    else
        tFinal = 600;
    end

    longOptions = odeset( ...
        'RelTol',1e-9, ...
        'AbsTol',1e-11, ...
        'MaxStep',0.02, ...
        'Events',@positiveCrossing);

    [~,~,te,xe] = ode113( ...
        @(t,x)vdpDynamics(t,x,mu), ...
        [0 tFinal], ...
        x0, ...
        longOptions);

    if numel(te) < 3
        error('Not enough positive crossings detected for mu=%g.',mu);
    end

    t1 = te(end-1);
    t2 = te(end);

    xCross = xe(end-1,:)';

    Tlong(i) = t2-t1;

    tPlotLong = linspace(0,Tlong(i),1800);

    orbitOptions = odeset( ...
        'RelTol',1e-9, ...
        'AbsTol',1e-11, ...
        'MaxStep',0.01);

    [~,Xlong] = ode113( ...
        @(t,x)vdpDynamics(t,x,mu), ...
        tPlotLong, ...
        xCross, ...
        orbitOptions);

    longOrbit{i} = Xlong;

    Along(i) = max(abs(Xlong(:,1)));


    %% --------------------------------------------------------
    % 3.2 Shooting method
    % ---------------------------------------------------------

    shootingGuess = [ ...
        xCross(2); ...
        Tlong(i)];

    shootFsolveOptions = optimoptions( ...
        'fsolve', ...
        'Display','off', ...
        'FunctionTolerance',1e-10, ...
        'StepTolerance',1e-10, ...
        'OptimalityTolerance',1e-10, ...
        'MaxIterations',200);

    zShoot = fsolve( ...
        @(z)shootResidual(z,mu,false), ...
        shootingGuess, ...
        shootFsolveOptions);

    vShoot = zShoot(1);
    Tshoot(i) = zShoot(2);

    if vShoot <= 0 || Tshoot(i) <= 0
        error('Invalid shooting solution for mu=%g.',mu);
    end

    tPlotShoot = linspace(0,Tshoot(i),1800);

    shootingODEOptions = odeset( ...
        'RelTol',1e-10, ...
        'AbsTol',1e-12, ...
        'MaxStep',0.005);

    [~,Xshoot] = ode113( ...
        @(t,x)vdpDynamics(t,x,mu), ...
        tPlotShoot, ...
        [0;vShoot], ...
        shootingODEOptions);

    shootOrbit{i} = Xshoot;

    Ashoot(i) = max(abs(Xshoot(:,1)));


    %% --------------------------------------------------------
    % 3.3 High-accuracy reference
    % ---------------------------------------------------------

    refGuess = [vShoot;Tshoot(i)];

    refFsolveOptions = optimoptions( ...
        'fsolve', ...
        'Display','off', ...
        'FunctionTolerance',1e-12, ...
        'StepTolerance',1e-12, ...
        'OptimalityTolerance',1e-12, ...
        'MaxIterations',400);

    zRef = fsolve( ...
        @(z)shootResidual(z,mu,true), ...
        refGuess, ...
        refFsolveOptions);

    vRef = zRef(1);
    Tref(i) = zRef(2);

    tPlotRef = linspace(0,Tref(i),2500);

    refODEOptions = odeset( ...
        'RelTol',1e-12, ...
        'AbsTol',1e-14, ...
        'MaxStep',0.002);

    [~,Xref] = ode113( ...
        @(t,x)vdpDynamics(t,x,mu), ...
        tPlotRef, ...
        [0;vRef], ...
        refODEOptions);

    referenceOrbit{i} = Xref;

    Aref(i) = max(abs(Xref(:,1)));


    %% --------------------------------------------------------
    % Display result
    % ---------------------------------------------------------

    fprintf('Reference period:      %.9f\n',Tref(i));
    fprintf('Long-time period:      %.9f\n',Tlong(i));
    fprintf('Shooting period:       %.9f\n',Tshoot(i));

    fprintf('Chapter 6 period:      %.9f\n', ...
        chapterReferencePeriod(i));

    fprintf('\n');

    fprintf('Reference amplitude:   %.9f\n',Aref(i));
    fprintf('Long-time amplitude:   %.9f\n',Along(i));
    fprintf('Shooting amplitude:    %.9f\n',Ashoot(i));

    fprintf('Chapter 6 amplitude:   %.9f\n', ...
        chapterReferenceAmplitude(i));

end


%% ============================================================
% 4. Numerical comparison table
% =============================================================

fprintf('\n====================================================\n');
fprintf('PERIOD COMPARISON\n');
fprintf('====================================================\n');

resultTable = table( ...
    muValues(:), ...
    Tref(:), ...
    Tlong(:), ...
    Tshoot(:), ...
    Aref(:), ...
    Along(:), ...
    Ashoot(:), ...
    'VariableNames',{ ...
        'mu', ...
        'ReferencePeriod', ...
        'LongTimePeriod', ...
        'ShootingPeriod', ...
        'ReferenceAmplitude', ...
        'LongTimeAmplitude', ...
        'ShootingAmplitude'});

disp(resultTable);


%% ============================================================
% 5. Figure 6.1
% =============================================================

% Unified font size for all visible figure text
fontSize = 20;

fig = figure( ...
    'Color','w', ...
    'Position',[80 150 1350 480]);

tl = tiledlayout(fig,1,3, ...
    'TileSpacing','compact', ...
    'Padding','compact');


for i = 1:nCases

    mu = muValues(i);

    Xref   = referenceOrbit{i};
    Xlong  = longOrbit{i};
    Xshoot = shootOrbit{i};

    ax = nexttile(tl);

    plot( ...
        Xref(:,1), ...
        Xref(:,2), ...
        '-', ...
        'LineWidth',2.4);

    hold on;

    plot( ...
        Xlong(:,1), ...
        Xlong(:,2), ...
        '--', ...
        'LineWidth',1.5);

    plot( ...
        Xshoot(:,1), ...
        Xshoot(:,2), ...
        ':', ...
        'LineWidth',1.8);

    grid on;
    box on;

    xlabel('$x_1$', ...
        'Interpreter','latex', ...
        'FontSize',fontSize);

    ylabel('$x_2$', ...
        'Interpreter','latex', ...
        'FontSize',fontSize);

    title( ...
        sprintf('$\\mu=%g$',mu), ...
        'Interpreter','latex', ...
        'FontSize',fontSize, ...
        'FontWeight','normal');

    xlim([-2.3 2.3]);

    if mu == 1
        ylim([-3.2 3.2]);
    elseif mu == 5
        ylim([-8.5 8.5]);
    else
        ylim([-15 15]);
    end

    set(ax, ...
        'FontName','Times New Roman', ...
        'FontSize',fontSize, ...
        'LineWidth',1);

    axis square;

end


%% ============================================================
% 6. Legend
% =============================================================

lgd = legend( ...
    {'High-accuracy reference', ...
     'Long-time integration', ...
     'Shooting method'}, ...
    'Orientation','horizontal');

lgd.Layout.Tile = 'south';

set(lgd, ...
    'FontName','Times New Roman', ...
    'FontSize',fontSize);


%% ============================================================
% 7. Overall title
% =============================================================

title( ...
    tl, ...
    'Van der Pol Periodic-Orbit Comparison', ...
    'FontName','Times New Roman', ...
    'FontSize',fontSize, ...
    'FontWeight','normal');


%% ============================================================
% 8. Export
% =============================================================

drawnow;

exportgraphics( ...
    tl, ...
    'Figure_6_1_VanDerPol_periodic_orbits.png', ...
    'Resolution',600);

exportgraphics( ...
    tl, ...
    'Figure_6_1_VanDerPol_periodic_orbits.pdf', ...
    'ContentType','vector');

fprintf('\nFigure 6.1 generated successfully.\n');
fprintf('Generated files:\n');
fprintf('Figure_6_1_VanDerPol_periodic_orbits.png\n');
fprintf('Figure_6_1_VanDerPol_periodic_orbits.pdf\n');

end


%% ============================================================
% Local function — Van der Pol dynamics
% =============================================================

function dx = vdpDynamics(~,x,mu)

x1 = x(1);
x2 = x(2);

dx = [ ...
    x2; ...
    mu*(1-x1^2)*x2-x1];

end


%% ============================================================
% Local function — positive crossing of x1 = 0
% =============================================================

function [value,isterminal,direction] = positiveCrossing(~,x)

value = x(1);

isterminal = 0;

direction = +1;

end


%% ============================================================
% Local function — shooting residual
% =============================================================

function F = shootResidual(z,mu,highAccuracy)

v0 = z(1);
T  = z(2);

if T <= 0

    F = [ ...
        1e3+abs(T); ...
        1e3+abs(T)];

    return;

end


if highAccuracy

    options = odeset( ...
        'RelTol',1e-12, ...
        'AbsTol',1e-14, ...
        'MaxStep',0.002);

else

    options = odeset( ...
        'RelTol',1e-10, ...
        'AbsTol',1e-12, ...
        'MaxStep',0.005);

end


[~,X] = ode113( ...
    @(t,x)vdpDynamics(t,x,mu), ...
    [0 T], ...
    [0;v0], ...
    options);

xEnd = X(end,:)';

F = [ ...
    xEnd(1); ...
    xEnd(2)-v0];

end