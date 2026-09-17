function dx = cstr_state_ct(x,u)
%CSTR_STATE_CT Continuous-time two-state non-isothermal CSTR.
%
% State order: [C_A; T]
% Input: T_c

p = cstr_params();

CA = x(1);
T  = x(2);
Tc = u(1);

% Guard only against a numerically invalid temperature during failed
% optimizer trial points. The physical benchmark operates far above zero K.
Texp = max(T,1.0);

r = p.k0 * exp(-p.E/(p.R*Texp)) * CA;

dCA = (p.F/p.V)*(p.CAf - CA) - r;

dT = (p.F/p.V)*(p.Tf - T) ...
    - (p.dH/p.rhoCp)*r ...
    - (p.UA/(p.rhoCp*p.V))*(T - Tc);

dx = [dCA; dT];
end
