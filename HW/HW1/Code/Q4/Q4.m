syms a b lambda tf l
eq1 = a * sinh((tf - b) / a) - l;
eq2 = lambda + a * cosh(b / a);
eq3 = lambda + a * cosh((tf - b) / a);
[solve_a, solve_b, solve_lambda] = ...
fzero([eq1, eq2, eq3], [a, b, lambda]);