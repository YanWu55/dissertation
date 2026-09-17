function xNext = cstr_state_dt(x,u)
%CSTR_STATE_DT Discrete CSTR-1 model using RK4 substeps.

p = cstr_params();
h = p.Ts / p.integrationSubsteps;

xNext = x(:);
for i = 1:p.integrationSubsteps
    k1 = cstr_state_ct(xNext,u);
    k2 = cstr_state_ct(xNext + 0.5*h*k1,u);
    k3 = cstr_state_ct(xNext + 0.5*h*k2,u);
    k4 = cstr_state_ct(xNext + h*k3,u);
    xNext = xNext + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end
end
