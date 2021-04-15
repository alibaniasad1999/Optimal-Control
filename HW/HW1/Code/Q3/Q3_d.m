syms c1 c2 c4 t0 t1 tf
eq1 = t0 + 1 / (2 * c1);
eq2 = t0^2 - c1 * t0 - c2;
eq3 = 2 * tf -c4 -1;
eq4 = 4 * t1 * (c1 + 1) - 16 * c1 - 16;
eq5 = c1 * t1 + c2 + t1 - c4;
eq6 = 2 * (t1 - 4)^2 - c1 * t1 - c2;
[solve_c1, solve_c2, solve_c4, solve_t0, solve_t1, solve_tf] = ...
vpasolve([eq1, eq2, eq3, eq4, eq5, eq6], [c1, c2 , c4, t0, t1, tf]);
% function in zero state %
theta_t0 = solve_t0.^2;
% function in final state %
theta_tf = solve_tf - 1;
% Function in Corner state %
theta_c = 4 + 2 * (solve_t1 - 4).^2;
r = (theta_t0 - theta_c).^2 + (theta_tf - theta_c).^2;
