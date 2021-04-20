syms y(x)
syms lambda
ode = (y - lambda) * diff(y, x, 2) + ...
    diff(y, x)^2 + diff(y, x)^4;
