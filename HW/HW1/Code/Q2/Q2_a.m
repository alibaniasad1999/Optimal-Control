syms x y
eq = x^2 + y^2 + sin(x)^2 + cos(y)^2 + 2 * sin(x) * cos(y);
% df/dx %
eq1 = diff(eq, x);
% df/dy %
eq2 = diff(eq, y);
% Gradiant f solver %
[sol_x, sol_y] = solve([eq1, eq2], [x, y]);
% calculate z with subject %
sol_z = sin(sol_x) + cos(sol_y);
% function %
f = sol_x^2 + sol_y^2 + sol_z^2;
% Distance %
r = sqrt(f);
rx = [0, sol_x];
ry = [0, sol_y];
rz = [0, sol_z];
[X, Y] = meshgrid(-1:.1:1);
Z = sin(X) + cos(Y);
[RX, RY] = meshgrid(-0.5787224241179413703304116508523:.1:0.5787224241179413703304116508523);
RZ = sin(RX) + cos(RY);
plot(rx, ry)
hold;
contour(X, Y, Z)
contour(RX, RY, RZ)