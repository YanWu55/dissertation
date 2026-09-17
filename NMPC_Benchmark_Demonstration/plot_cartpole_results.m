function plot_cartpole_results(result,outdir)
%PLOT_CARTPOLE_RESULTS Create dissertation-ready CP-1 figures.

p = result.cfg.p;
t = result.t;
tu = result.tu;
x = result.x;
u = result.u;

% Figure 1: state response
f1 = figure('Color','w','Name','CP-1 state response');
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t,x(1,:),'LineWidth',1.4);
hold on;
yline(0,'--','Target');
yline(p.zMax,':','Track limit');
yline(p.zMin,':','Track limit');
xlabel('Time (s)');
ylabel('Cart position z (m)');
grid on;

nexttile;
plot(t,rad2deg(x(2,:)),'LineWidth',1.4);
hold on;
yline(0,'--','Upright target');
xlabel('Time (s)');
ylabel('Pole angle \theta (deg)');
grid on;

nexttile;
plot(t,x(3,:),'LineWidth',1.4);
hold on; yline(0,'--');
xlabel('Time (s)');
ylabel('Cart velocity (m/s)');
grid on;

nexttile;
plot(t,rad2deg(x(4,:)),'LineWidth',1.4);
hold on; yline(0,'--');
xlabel('Time (s)');
ylabel('Angular velocity (deg/s)');
grid on;

title(tl,'CP-1: Cart-Pole Closed-Loop State Response');
exportgraphics(f1,fullfile(outdir,'CP1_state_response.png'),'Resolution',200);

% Figure 2: control + phase portrait
f2 = figure('Color','w','Name','CP-1 input and phase portrait');
tl2 = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

nexttile;
stairs(tu,u,'LineWidth',1.4);
hold on;
yline(p.uMax,':','Input limit');
yline(p.uMin,':','Input limit');
xlabel('Time (s)');
ylabel('Force F (N)');
grid on;

nexttile;
plot(rad2deg(x(2,:)),rad2deg(x(4,:)),'LineWidth',1.4);
hold on;
plot(0,0,'o','MarkerSize',7,'LineWidth',1.3);
xlabel('Pole angle \theta (deg)');
ylabel('Angular velocity (deg/s)');
grid on;

title(tl2,'CP-1: NMPC Input and Pole Phase Portrait');
exportgraphics(f2,fullfile(outdir,'CP1_input_phase.png'),'Resolution',200);

% Figure 3: solver diagnostics
f3 = figure('Color','w','Name','CP-1 solver diagnostics');
tl3 = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(tu,result.solveTime,'LineWidth',1.2);
xlabel('Time (s)');
ylabel('NMPC solve time (s)');
grid on;

nexttile;
stairs(tu,result.exitFlag,'LineWidth',1.2);
xlabel('Time (s)');
ylabel('ExitFlag');
grid on;

title(tl3,'CP-1: NMPC Solver Diagnostics');
exportgraphics(f3,fullfile(outdir,'CP1_solver_diagnostics.png'),'Resolution',200);
end
