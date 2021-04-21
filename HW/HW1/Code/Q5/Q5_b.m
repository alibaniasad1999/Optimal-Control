syms x(t) c
dotx = diff(x, t);
ode(t) = dotx^2 + dotx * ((2 - c) / (2 * t)) == 0;
xSol(t) = dsolve(ode);