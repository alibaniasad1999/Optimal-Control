syms y(x)
Dy = diff(y);
syms lambda
ode = (y - lambda) * diff(y, x, 2) + ...
    diff(y, x)^2 + diff(y, x)^4;
