syms x(t) C1 C2 C3 C4
ode(t) = x == diff(x, t, 4);
xSol = dsolve(ode);
diff(xSol, t, 2)
