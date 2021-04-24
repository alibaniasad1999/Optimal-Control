syms x(t) c tf
ode(t) = 1/(x * sqrt(1 + diff(x, t)^2)) - c;
eq1 = (tf - 9)^2 + (-tf * (c * tf +2)) / c                  == 9;
eq2 = (tf - 9)^2 * (c * tf - 1)^2 / (c * tf * (c * tf - 2)) == 9;
[sol_c, sol_tf] = vpasolve([eq1, eq2], [c, tf]);
C = double(sol_c(6));
t = 0:.01:double(sol_tf(6));
X = sqrt(-C * t .* (C * t + 2)) / C;
figure1 = figure('Name','Figure','NumberTitle','off');
hold on;
r = 3;
alpha = 0:.01:2*pi;
plot(9 + r * sin(alpha), r * cos(alpha), 'linewidth', 2);
plot(t, X, 'linewidth', 2);
plot(t, -X, 'linewidth', 2);
axis equal
xlabel('time')
ylabel('x');
axis([0 12 -6 6]);
legend('Final $\theta(t)$', 'Minimum Cost', 'Maximum Cost', 'Interpreter','latex')
print(figure1, 'Figure5fFix.png','-dpng','-r300');