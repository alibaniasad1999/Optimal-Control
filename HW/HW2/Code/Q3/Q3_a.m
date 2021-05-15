syms x1(t) x2(t) p1(t) p2(t)
ode1 = diff(x1) == x2;
ode2 = diff(x2) == -x1 - 0.1 * x2 - 0.5 * p2;
ode3 = diff(p1) == -2 * x1 + p2;
ode4 = diff(p2) == -2 * x2 - p1 + 0.1 * p2;
odes = [ode1; ode2; ode3; ode4];
cond1 = x1(0) == 1;
cond2 = x2(0) == 1;
cond3 = x1(3) == 0;
cond4 = x2(3) == 0;
conds = [cond1; cond2; cond3; cond4];
[x1Sol(t), x2Sol(t), p1Sol(t), p2Sol(t)] = dsolve(odes,conds);
%[x1Sol(t), x2Sol(t), p1Sol(t), p2Sol(t)] = dsolve(odes);