syms x(t) lambda c1 c2
ode = diff(x, t, 2) - 2 * x + lambda;
xSol = dsolve(ode);
eq1 = c1 + c2 + lambda/2;
eq2 = c1 * exp(sqrt(2)) + c2 * exp(-sqrt(2)) + lambda / 2 == 1;
eq3 = sqrt(2) / 2 * c1 * (exp( sqrt(2)) - 1) + ...
     -sqrt(2) / 2 * c2 * (exp(-sqrt(2)) - 1) + lambda / 2 == 1;
 [sol_c1, sol_c2, sol_lambda] = ...
     solve([eq1, eq2, eq2], [c1, c2, lambda]);