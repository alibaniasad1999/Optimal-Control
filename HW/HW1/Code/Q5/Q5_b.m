syms x(t)
dotx = diff(x, t);
ddotx = diff(x, t, 2);
eq(t) = 2 * t * dotx + 2 / dotx;
ode2(t) = (2 * t - 2 / (dotx^2)) * ddotx + ...
          (2 * dotx + 2 * t * ddotx - 2 * ddotx / (dotx^2)) * dotx;
ySol(t) = dsolve(ode2, x(0) == 1, x(2) == 2);