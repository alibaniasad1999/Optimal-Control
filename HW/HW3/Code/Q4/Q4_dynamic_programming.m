%%%%%%%%%% Q4 Dynamic programming %%%%%%%%%%
x_1 = -3:0.05:3;
x_2 = -1:0.01:1;
x_1_max =  3;
x_1_min = -3;
x_2_max =  1;
x_2_min = -1;
time = 0:0.05:5;
delta_t = 0.05;
u = -0.8:0.001:0.8;
min_cost  = inf;
u_matrix = zeros(length(x_1), length(x_2), length(time));
tic
for i = 1:length(x_1)
    for j = 1:length(x_2)
        for k = 1:length(time)
            for m = 1:length(u)
                [x_1_new, x_2_new] = state(x_1(i), x_2(j), u(m), delta_t);
                if x_1_new > x_1_max || x_1_new < x_1_min || x_2_new > x_2_max ||...
                        x_2_new < x_2_min
                    continue
                end
                if min_cost > cost(x_1_new, x_2_new, delta_t, u(m))
                    min_cost = cost(x_1_new, x_2_new, delta_t, u(m));
                    u_matrix(i, j, k) = u(m);
                end
            end
            min_cost = inf;
        end
    end
    toc
end
toc
function j_cost = cost(x_1, x_2, delta_t, u)
j_cost = (x_1^2 + x_2^2 + u^2) * delta_t;
end
function [x_1_new, x_2_new] = state(x_1, x_2, u, delta_t)
x_1_new = x_2 * delta_t + x_1;
x_2_new = (-0.4 * x_1 - 0.2 * x_2 ^ 2 + u) * delta_t + x_2;
end
