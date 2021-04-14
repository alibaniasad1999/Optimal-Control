syms x y
eq = x^2 + y^2 + sin(x)^2 + cos(y)^2 + 2 * sin(x) * cos(y);
% df/dx %
eq1 = diff(eq, x);
% df/dy %
eq2 = diff(eq, y);
% Gradiant f solver %