syms x1 x2 y1 y2 lambda1 lambda2
Lagrange = (x1-x2)^2 + (y1-y2)^2 + lambda1 * (y1 - x1^2) + ...
           lambda2 * (y2 - x2 + 1);
eq1 = diff(Lagrange, x1);
eq2 = diff(Lagrange, y1);
eq3 = diff(Lagrange, x2);
eq4 = diff(Lagrange, y2);
eq5 = diff(Lagrange, lambda1);
eq6 = diff(Lagrange, lambda2);
[sol_x1, sol_y1, sol_x2, sol_y2, sol_lambda1, sol_lambda2] = ... 
solve([eq1, eq2, eq3, eq4, eq5, eq6], [x1, y1, x2, y2, lambda1, lambda2]);