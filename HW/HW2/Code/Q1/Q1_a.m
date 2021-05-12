% declare variables %
syms p(t)
% Differentia equation %
ode = diff(p) == 0.1 * p + 1;
% Answer %
pSol = dsolve(ode);