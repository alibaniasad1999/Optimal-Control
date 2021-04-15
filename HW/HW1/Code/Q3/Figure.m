% Question 3 figure %
% time %
t = -5:.01:5;
% function in zero state %
theta_t0 = t.^2;
% function in zero state %
theta_tf = t - 1;
% Answer function %
t_ans = 0.5:.001:0.875;
answer_function = -t_ans + 0.75;
% figure %
figure1 = figure('NumberTitle','off');
hold on
plot(t, theta_t0, 'linewidth', 2);
plot(t, theta_tf, 'linewidth', 2);
plot(t_ans, answer_function, 'linewidth', 2);
axis equal
axis([-1 1 -1 1])
legend('Initial time $\theta(t)$', 'Final time $\theta(t)$', 'Answer', 'Interpreter','latex')
xlabel('time', 'Interpreter','latex')
ylabel('x(t)', 'Interpreter','latex')
print(figure1, 'Q3figure.png','-dpng','-r300');

