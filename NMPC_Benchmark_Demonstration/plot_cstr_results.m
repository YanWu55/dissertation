function plot_cstr_results(result,outdir)
%PLOT_CSTR_RESULTS Create dissertation-ready CSTR-1 figures.

p = result.cfg.p;
t = result.t;
tu = result.tu;
x = result.x;
u = result.u;

% Figure 1: states
f1 = figure('Color','w','Name','CSTR-1 state response');
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t,x(1,:),'LineWidth',1.4);
hold on;
yline(p.CAstar,'--','Target');
yline(p.CAMin,':','Lower limit');
yline(p.CAMax,':','Upper limit');
xlabel('Time (h)');
ylabel('C_A (kmol/m^3)');
grid on;

nexttile;
plot(t,x(2,:),'LineWidth',1.4);
hold on;
yline(p.Tstar,'--','Target');
yline(p.TMin,':','Lower limit');
yline(p.TMax,':','Upper limit');
xlabel('Time (h)');
ylabel('Reactor temperature T (K)');
grid on;

title(tl,'CSTR-1: Closed-Loop State Response');
exportgraphics(f1,fullfile(outdir,'CSTR1_state_response.png'),'Resolution',200);

% Figure 2: input + phase trajectory
f2 = figure('Color','w','Name','CSTR-1 input and state-space trajectory');
tl2 = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

nexttile;
stairs(tu,u,'LineWidth',1.4);
hold on;
yline(p.Tcstar,'--','Steady input');
yline(p.TcMin,':','Input limit');
yline(p.TcMax,':','Input limit');
xlabel('Time (h)');
ylabel('Coolant temperature T_c (K)');
grid on;

nexttile;
plot(x(1,:),x(2,:),'LineWidth',1.4);
hold on;
plot(p.x0(1),p.x0(2),'o','MarkerSize',7,'LineWidth',1.3);
plot(p.CAstar,p.Tstar,'x','MarkerSize',9,'LineWidth',1.5);
xlabel('C_A (kmol/m^3)');
ylabel('T (K)');
grid on;

title(tl2,'CSTR-1: Control Input and State-Space Trajectory');
exportgraphics(f2,fullfile(outdir,'CSTR1_input_phase.png'),'Resolution',200);

% Figure 3: solver diagnostics
f3 = figure('Color','w','Name','CSTR-1 solver diagnostics');
tl3 = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(tu,result.solveTime,'LineWidth',1.2);
xlabel('Time (h)');
ylabel('NMPC solve time (s)');
grid on;

nexttile;
stairs(tu,result.exitFlag,'LineWidth',1.2);
xlabel('Time (h)');
ylabel('ExitFlag');
grid on;

title(tl3,'CSTR-1: NMPC Solver Diagnostics');
exportgraphics(f3,fullfile(outdir,'CSTR1_solver_diagnostics.png'),'Resolution',200);
end
