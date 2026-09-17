function result = run_cstr_demo(outdir)
%RUN_CSTR_DEMO Run CSTR-1 nonlinear MPC closed-loop demonstration.

if nargin < 1
    outdir = fullfile(pwd,'results');
end
if ~exist(outdir,'dir')
    mkdir(outdir);
end

[nlobj,cfg] = setup_cstr_nmpc();
p = cfg.p;

N = round(cfg.Tsim/p.Ts);
t = (0:N)*p.Ts;

x = zeros(2,N+1);
u = zeros(1,N);
solveTime = zeros(1,N);
exitFlag = zeros(1,N);
iterations = zeros(1,N);
predictedCost = nan(1,N);

x(:,1) = cfg.x0;
lastMV = cfg.u0;
opt = nlmpcmoveopt;

for k = 1:N
    tic;
    [mv,opt,info] = nlmpcmove(nlobj,x(:,k),lastMV,cfg.yref,[],opt);
    solveTime(k) = toc;

    u(k) = mv(1);
    exitFlag(k) = info.ExitFlag;
    iterations(k) = info.Iterations;
    if info.ExitFlag >= 0
        predictedCost(k) = info.Cost;
    end

    if info.ExitFlag < 0
        warning('CSTR-1:SolverFailure', ...
            'NMPC solver returned ExitFlag=%d at t=%.3f h. Previous MV is used.', ...
            info.ExitFlag,t(k));
    end

    x(:,k+1) = cstr_state_dt(x(:,k),mv);
    lastMV = mv;
end

result.id = cfg.id;
result.t = t;
result.tu = t(1:end-1);
result.x = x;
result.u = u;
result.solveTime = solveTime;
result.exitFlag = exitFlag;
result.iterations = iterations;
result.predictedCost = predictedCost;
result.cfg = cfg;

result.metrics = cstr_metrics(result);

disp('CSTR-1 performance metrics:');
disp(struct2table(result.metrics));

writetable(struct2table(result.metrics), ...
    fullfile(outdir,'CSTR1_metrics.csv'));

save(fullfile(outdir,'CSTR1_result.mat'),'result');

plot_cstr_results(result,outdir);
end
