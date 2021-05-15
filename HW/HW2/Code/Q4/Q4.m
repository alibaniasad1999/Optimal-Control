syms x1(t) x2(t)
ode1 = diff(x1) == -x1 + 1;
ode2 = diff(x2) == -x2 + 1;
odes = [ode1; ode2];
[x1Sol, x2Sol] = dsolve(odes);