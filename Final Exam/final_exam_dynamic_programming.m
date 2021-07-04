%%%%%%%%%% Exam question part b Dynamic programming %%%%%%%%%%
x_1 = -1:0.01:3;
x_2 = -1:0.01:1;
x_1_max =  3;
x_1_min = -1;
x_2_max =  1;
x_2_min = -1;
time = 0:.05:7;
delta_t = 1;
u = -1:0.005:1;
min_cost  = inf;
contrl_law_matrix = zeros(length(x_1), length(x_2), length(time));
cost_matrix = ones(length(x_1), length(x_2)) * inf;
tic
for k = 1:length(time)
    for i = 1:length(x_1)
        for j = 1:length(x_2)
            for m = 1:length(u)
                [x_1_new, x_2_new] = state(x_1(i), x_2(j), u(m), delta_t);
                
                if x_1_new > x_1_max || x_1_new < x_1_min || x_2_new > x_2_max ||...
                    x_2_new < x_2_min
                    continue
                end
                J_cost = cost(x_1_new, x_2_new, delta_t, u(m));
                if min_cost > J_cost
                    min_cost = J_cost;
                    u_matrix(i, j, k) = u(m);
                end
            end
            min_cost = inf;
        end
    end
    toc
end
toc
X = zeros(2, length(time));
x0 = [1; 0.2];
X(:, 1) = x0;
for i = 2:length(X)
    [x_1_i, x_2_i] = interpolation(X(1, i-1), X(2, i-1), x_1, x_2);
    u_i = u_matrix(x_1_i, x_2_i, i-1);
    [X(1, i), X(2, i)] = state(X(1, i-1), X(2, i-1), u_i, delta_t);
end
plot(time, X(1, :))
hold
plot(time, X(2, :))
function j_cost = cost(x_1, x_2, delta_t, u)
j_cost = (x_1^2 + x_2^2 + u^2) * delta_t;
end
function [x_1_new, x_2_new] = state(x_1, x_2, u, delta_t)
x_1_new = x_2 * delta_t + x_1;
x_2_new = (-0.4 * x_1 - 0.2 * x_2 ^ 2 + u) * delta_t + x_2;
end
function [x_1, x_2] = interpolation(x_1_i, x_2_i, x_1_array, x_2_array)
[~, i] = min(abs(x_1_i - x_1_array));
x_1 = i;
[~, j] = min(abs(x_2_i - x_2_array));
x_2 = j;
end

