%% SMOKE_TEST_MODELS
% Quick model-only checks. Does not run the full closed loop.

clear;
clc;
check_requirements();

fprintf('\n--- Cart-Pole model check ---\n');
pc = cartpole_params();
fprintf('dx at upright equilibrium with u=0:\n');
disp(cartpole_state_ct(pc.xref,0));

[nlcp,cfgcp] = setup_cartpole_nmpc(); %#ok<ASGLU>

fprintf('\n--- CSTR model check ---\n');
pr = cstr_params();
fprintf('Exact CSTR target equilibrium:\n');
fprintf('CA*=%.10f, T*=%.10f, Tc*=%.10f\n',pr.CAstar,pr.Tstar,pr.Tcstar);
fprintf('dx at target equilibrium:\n');
disp(cstr_state_ct(pr.xref,pr.Tcstar));

[nlcstr,cfgcstr] = setup_cstr_nmpc(); %#ok<ASGLU>

fprintf('\nAll model validation calls completed.\n');
