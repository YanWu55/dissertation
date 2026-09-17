function xNext = cartpole_state_dt(x,u)
%CARTPOLE_STATE_DT Discrete CP-1 model using RK4 substeps.

p = cartpole_params();
h = p.Ts / p.integrationSubsteps;

xNext = x(:);
for i = 1:p.integrationSubsteps
    k1 = cartpole_state_ct(xNext,u);
    k2 = cartpole_state_ct(xNext + 0.5*h*k1,u);
    k3 = cartpole_state_ct(xNext + 0.5*h*k2,u);
    k4 = cartpole_state_ct(xNext + h*k3,u);
    xNext = xNext + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end
end
