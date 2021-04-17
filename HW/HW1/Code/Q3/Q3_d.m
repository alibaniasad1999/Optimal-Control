syms c1 c2 c4 t0 t1 tf
eq1 = t0 + 1 / (2 * c1);
eq2 = t0^2 - c1 * t0 - c2;
eq3 = 2 * tf -c4 -1;
eq4 = 4 * t1 * (c1 + 1) - 16 * c1 - 16;
eq5 = c1 * t1 + c2 + t1 - c4;
eq6 = 2 * (t1 - 4)^2 - c1 * t1 - c2 + 4;
[solve_c1, solve_c2, solve_c4, solve_t0, solve_t1, solve_tf] = ...
vpasolve([eq1, eq2, eq3, eq4, eq5, eq6], [c1, c2 , c4, t0, t1, tf]);
% function in zero state %
theta_t0 = solve_t0.^2;
% function in final state %
theta_tf = solve_tf - 1;
% Function in Corner state %
theta_c = 4 + 2 * (solve_t1 - 4).^2;
r = (theta_t0 - theta_c).^2 + (theta_tf - theta_c).^2;
% Question 3 figure %
% time %
t = -10:.01:10;
% function in zero state %
theta_t0 = t.^2;
% function in zero state %
theta_tf = t - 1;
% function in corner %
theta_c = 4+2*(t-4).^2;
% Answer function %
t_ans = 0.5:.001:0.875;
answer_function = -t_ans + 0.75;
time1 = solve_t0(5):0.01:solve_t1(5);
time2 = solve_t1(5):0.01:solve_tf(5);
f1 = solve_c1(5) * time1 + solve_c2(5);
f2 = -1 * time2 + solve_c4(5);
% figure %
figure1 = figure('NumberTitle','off');
hold on
plot(t, theta_t0, 'linewidth', 2);
plot(t, theta_tf, 'linewidth', 2);
plot(t, theta_c, 'linewidth', 2);
plot(t_ans, answer_function, 'linewidth', 2);
plot(time1, f1, 'linewidth', 2);
plot(time2, f2, 'linewidth', 2);
axis equal
axis([-2 6 -2 6])
legend('Initial time $\theta(t)$', 'Final time $\theta(t)$'...
    , 'Corner time $\theta(t)$', 'Answer', 'Answer with corner I'...
    , 'Answer with corner II', 'Interpreter','latex')
xlabel('time', 'Interpreter','latex')
ylabel('x(t)', 'Interpreter','latex')
print(figure1, 'Q3WithCornerFigure.png','-dpng','-r300');

