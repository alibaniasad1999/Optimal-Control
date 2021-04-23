syms x(t) c tf
ode(t) = 1/(x * sqrt(1 + diff(x, t)^2)) - c;
eq1 = (tf - 9)^2 + (-tf * (c * tf +2)) / c                    == 9;
eq2 = (tf - 9)^2 * (c * tf - 1)^2 / (c * tf * (c * tf - 2)) == 9;
[sol_c, sol_tf] = vpasolve([eq1, eq2], [c, tf]);