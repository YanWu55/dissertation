function dx = cartpole_state_ct(x,u)
%CARTPOLE_STATE_CT Continuous-time nonlinear cart-pole dynamics.
%
% theta = 0 is upright.
% State order: [z; theta; z_dot; theta_dot]

p = cartpole_params();

z_dot = x(3);
theta = x(2);
theta_dot = x(4);
F = u(1);

s = sin(theta);
c = cos(theta);

den = p.M + p.m*s^2;

z_ddot = ( ...
    F - p.Kd*z_dot ...
    - p.m*p.l*theta_dot^2*s ...
    + p.m*p.g*s*c ) / den;

theta_ddot = ( ...
    p.g*s ...
    + (F - p.Kd*z_dot - p.m*p.l*theta_dot^2*s)*c/(p.M+p.m) ...
    ) / (p.l - p.m*p.l*c^2/(p.M+p.m));

dx = [z_dot; theta_dot; z_ddot; theta_ddot];
end
