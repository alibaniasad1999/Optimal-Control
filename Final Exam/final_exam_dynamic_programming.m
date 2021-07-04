%%%%%%%%%% Exam question part b Dynamic programming %%%%%%%%%%
x_1       = -1:   1:3;
x_1_inter = -1:.001:3; % for interpolation
x_2 =       -1:  .5:1;
x_2_inter = -1:.005:1; % for interpolation
x_2_max =  1;
x_2_min = -1;
time = 0:1:3;
delta_t = 1;
u = -1:.5:1;
contrl_law_matrix = zeros(length(x_1), length(x_2), length(time)-1);
cost_matrix = ones(length(x_1), length(x_2), length(time)-1) * inf;
tic
for k = length(time)-1:-1:1
    for i = 1:length(x_1)
        for j = 1:length(x_2)
            for m = 1:length(u)
                [x_1_new, x_2_new] = state(x_1(i), x_2(j), u(m), delta_t);
                if x_2_new > x_2_max || x_2_new < x_2_min
                    continue
                end
                if k == length(time)-1
                    J_cost = cost(x_1_new, x_2_new, delta_t, u(m)); % integral cost
                    J_cost = J_cost + 5 * (x_1_new^2 + x_2_new^2);
                else
                    J_cost = cost(x_1_new, x_2_new, delta_t, u(m)); % integral cost
                    J_cost = J_cost + cost_interpolation(x_1_new, x_2_new, x_1,...
    x_2, cost_matrix(:, :, k+1));
                end
                if cost_matrix(i, j, k) > J_cost
                    cost_matrix(i, j, k) = J_cost;
                    u_matrix(i, j, k) = u(m);
                end
            end
        end
    end
    toc
end
toc
X = zeros(2, length(time));
x0 = [2; 0];
X(:, 1) = x0;
for i = 1:length(X)-1
    [x_1_i, x_2_i] = interpolation(X(1, i), X(2, i), x_1, x_2);
    u_i = u_matrix(x_1_i, x_2_i, i);
    [X(1, i+1), X(2, i+1)] = state(X(1, i), X(2, i), u_i, delta_t);
end
plot(time, X(1, :))
hold
plot(time, X(2, :))
function j_cost = cost(x_1, x_2, delta_t, u)
j_cost = (x_1^2 + x_2^2 + u^2) * delta_t;
end
function [x_1_new, x_2_new] = state(x_1, x_2, u, delta_t)
x_2_new = (-0.4 * x_1 - 0.2 * x_2 ^ 2 + u) * delta_t + x_2;
x_1_new = x_2 * delta_t + x_1;
end
function [x_1, x_2] = interpolation(x_1_i, x_2_i, x_1_array, x_2_array)
[~, i] = min(abs(x_1_i - x_1_array));
x_1 = i;
[~, j] = min(abs(x_2_i - x_2_array));
x_2 = j;
end
function cost_ij = cost_interpolation(x_1_i, x_2_i, x_1_array,...
    x_2_array, cost_array)
x_1_new = linspace(x_1_array(1), x_1_array(end), 100 * length(x_1_array));
x_2_new = linspace(x_2_array(1), x_2_array(end), 100 * length(x_2_array));
[x, y] = meshgrid(x_1_array, x_2_array);
[xx, yy] = meshgrid(x_1_new, x_2_new);
cost = interp2(x, y, cost_array, xx, yy);
[~, i] = min(abs(x_1_i - x_1_new));
[~, j] = min(abs(x_2_i - x_2_new));
cost_ij = cost(i, j);
end

