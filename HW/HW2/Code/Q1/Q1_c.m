% declare variables %
syms p(t)
% Differentia equation %
ode = diff(p) == 0.1 * p;
% Answer %
pSol = dsolve(ode);