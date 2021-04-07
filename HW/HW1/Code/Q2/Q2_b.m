syms x y z lambda
Lagrange = x^2 + y^2 + z^2 + lambda * (sin(x) + cos(y) - z);
eq1 = diff(Lagrange, x);
eq2 = diff(Lagrange, y);
eq3 = diff(Lagrange, z);
eq4 = diff(Lagrange, lambda);
[sol_x, sol_y, sol_z, sol_lambda] = solve([eq1, eq2, eq3, eq4], [x, y, z, lambda]);