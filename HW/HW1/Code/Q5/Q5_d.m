syms x(t)
eq(t) = diff(x, t, 4) - 2 * diff(x, t, 2) + x;
xSol(t) = dsolve(eq);