%%%%%%%%%% Exam question part b Dynamic programming %%%%%%%%%%
%% Control Law
x_1  =  -1.4:0.2:1.4;
x_2  =  -1.4:0.2:1.4;
delta_t = .05;
time = 0:delta_t:5;
u = -.8:.8:.8;
% contrl_law_matrix = zeros(length(x_1), length(x_2), length(time)-1);
cost_matrix = ones(length(x_1), length(x_2), length(time)-1) * inf;
tic
check_valid = false;
u_matrix = NaN(length(x_1), length(x_2), length(time));
cost_u = length(u);
for k = length(time)-1:-1:1
    for i = 1:length(x_1)
        for j = 1:length(x_2)
            for m = 1:length(u)
                [x_1_new, x_2_new] = state(x_1(i), x_2(j), u(m), delta_t);
                if k == length(time)-1
                    J_cost = cost(x_1_new, x_2_new, delta_t, u(m)); % integral cost
                    J_cost = J_cost + 10 * (x_1_new^2 + x_2_new^2);
                else
                    J_cost = cost(x_1_new, x_2_new, delta_t, u(m)); % integral cost
                    J_cost = J_cost + interpolation(x_1_new, x_2_new, x_1,...
    x_2, cost_matrix(:, :, k+1));
                end
                cost_u(m) = J_cost;
            end
            [cost_matrix(i, j, k), u_matrix(i, j, k)] = u_iterpolation(u, cost_u);
        end
    end
    fprintf('Estimated time at time %1.2f second = %1.4f sec\n', time(k), toc)
end
toc
contrl_law_matrix = u_matrix;
%% Find Way
save control_law.mat control_law_matrix
X = zeros(2, length(time));
x0 = [1; 0.2];
X(:, 1) = x0;
control = zeros(length(X), 1);
u_matrix = contrl_law_matrix;
for i = 1:length(X)-1
    u_i = interpolation(X(1, i), X(2, i), x_1, x_2, u_matrix(:, :, i));
    control(i) = u_i;
    [X(1, i+1), X(2, i+1)] = state(X(1, i), X(2, i), u_i, delta_t);
end
plot(time, X , 'linewidth', 2)
hold on
xlabel('$Time_{\sec}$', 'interpreter', 'latex');
ylabel('$\vec{X}$', 'interpreter', 'latex');
legend('$x$', '$\dot{x}$', 'interpreter', 'latex');
print('../../Figure/Q4/DP.png','-dpng','-r300')
function j_cost = cost(x_1, x_2, delta_t, u)
j_cost = (x_1^2 + x_2^2 + u^2) * delta_t;
end
function [x_1_new, x_2_new] = state(x_1, x_2, u, delta_t)
x_1_new = x_2 * delta_t + x_1;
x_2_new = (-0.4 * x_1 - 0.2 * x_2 ^ 2 + u) * delta_t + x_2;
end
function cost_ans = interpolation(x_1_i, x_2_i, x_1_array,...
    x_2_array, cost_array)
x_1_new = linspace(x_1_array(1), x_1_array(end), 1000);
x_2_new = linspace(x_2_array(1), x_2_array(end), 1000);
[x, y] = meshgrid(x_1_array, x_2_array);
cost_ij = cost_array();
x_1_new = x_1_new';
x_2_new = x_2_new';
[xx, yy] = meshgrid(x_1_new, x_2_new);
cost = interp2(x, y, cost_ij, xx, yy);
[~, i] = min(abs(x_1_i - x_1_new));
[~, j] = min(abs(x_2_i - x_2_new));
cost_ans = cost(i, j);
end
function [cost_opt, u] = u_iterpolation(u, cost)
u_new = -0.8:0.001:0.8;
cost_new = interp1(u, cost, u_new);
[cost, index] = min(cost_new);
cost_opt = cost;
u = u_new(index);
end