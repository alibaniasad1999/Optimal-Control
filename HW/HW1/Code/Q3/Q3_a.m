syms x1 x2
eq = (x1 - x2)^2 + (x1^2 - (x2-1))^2;
eq1 = diff(eq, x1);
eq2 = diff(eq, x2);
[sol_x, sol_y] = solve([eq1, eq2], [x1, x2]);