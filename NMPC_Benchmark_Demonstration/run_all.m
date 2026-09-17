%% RUN_ALL
% One-command tutorial demonstration:
%   NMPC -> Cart-Pole (CP-1) + CSTR (CSTR-1)
%
% Outputs are written to ./results
%
% This script requires:
%   - Model Predictive Control Toolbox
%   - Optimization Toolbox

clear;
clc;
close all;

check_requirements();

outdir = fullfile(pwd,'results');
if ~exist(outdir,'dir')
    mkdir(outdir);
end

fprintf('\n============================================================\n');
fprintf(' NMPC BENCHMARK DEMONSTRATION\n');
fprintf(' Same algorithm -> Cart-Pole + CSTR\n');
fprintf('============================================================\n\n');

fprintf('Running CP-1 (Cart-Pole)...\n');
cp = run_cartpole_demo(outdir);

fprintf('\nRunning CSTR-1...\n');
cstr = run_cstr_demo(outdir);

fprintf('\nCreating cross-system comparison...\n');

System = ["Cart-Pole (CP-1)"; "CSTR (CSTR-1)"];
NormalizedRMSE = [cp.metrics.NormalizedRMSE; cstr.metrics.NormalizedRMSE];
SettlingTime = [cp.metrics.SettlingTime; cstr.metrics.SettlingTime];
ControlEffort = [cp.metrics.ControlEffort; cstr.metrics.ControlEffort];
MaxStateViolation = [cp.metrics.MaxStateViolation; cstr.metrics.MaxStateViolation];
MaxInputViolation = [cp.metrics.MaxInputViolation; cstr.metrics.MaxInputViolation];
ClosedLoopCost = [cp.metrics.ClosedLoopCost; cstr.metrics.ClosedLoopCost];
MeanSolveTime = [cp.metrics.MeanSolveTime; cstr.metrics.MeanSolveTime];
MaxSolveTime = [cp.metrics.MaxSolveTime; cstr.metrics.MaxSolveTime];
SolverSuccessRate = [cp.metrics.SolverSuccessRate; cstr.metrics.SolverSuccessRate];

comparison = table(System,NormalizedRMSE,SettlingTime,ControlEffort, ...
    MaxStateViolation,MaxInputViolation,ClosedLoopCost, ...
    MeanSolveTime,MaxSolveTime,SolverSuccessRate);

disp(comparison);
writetable(comparison,fullfile(outdir,'cross_system_comparison.csv'));

save(fullfile(outdir,'all_results.mat'),'cp','cstr','comparison');

fprintf('\n============================================================\n');
fprintf(' Finished.\n');
fprintf(' Results folder: %s\n', outdir);
fprintf('============================================================\n');
