syms p(t)
ode = diff(p) == -0.1 * p - 1;
pSol = dsolve(ode);