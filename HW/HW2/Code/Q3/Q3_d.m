%% ODE Question 3 part d %%
global A B Q R R_inv n H
A = [ 0    1  ;
     -1 -0.1] ;
B = [ 0    1]';
Q = [ 1    0  ;
      0    1] ;
R = 1;
R_inv = R^-1;
n = 2;
H = eye(2);
tf = 10;
%%%% initial condition %%%%
x0	= [1 1]';
K0 = H;
%%%% ODE %%%%
global t_K K_arr
alpha = [1 5 10];
for i = 1:length(alpha)
    %%% initial alpha and Q %%%
    Q = alpha(i) * Q;
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    %%% solve System equation %%%
    [t, x] = ode45(@diff_eq_states, [0, tf], x0);
    %%% K %%%
    figure_name = strcat('System response (alpha = ', num2str(alpha(i)), ')');
    figure1K = figure('Name',figure_name, 'NumberTitle','off');
    plot(t_K, K_arr);
    xlabel('time');
    ylabel('K');
    legend('$K_1$', '$K_2$', '$K_3$', '$K_4$', 'Interpreter','latex');
    figure_save_name = append('figures/Kalpha', num2str(alpha(i)), '.png');
    print(figure1K, figure_save_name,'-dpng','-r400');
    close;
    %%% u %%%
    figure1u = figure('Name',figure_name, 'NumberTitle','off');
    %%% calculate u(t) %%%
    ue = zeros(1);
    K_t	= interp1(t_K, K_arr, t);
    for j = 1:length(t)
        ue(j) = -R_inv*B'*reshape(K_t(j, :), n, n)*x(j,:)';
    end
    plot(t, ue);
    xlabel('time');
    ylabel('u');
    figure_save_name = append('figures/ualpha', num2str(alpha(i)), '.png');
    print(figure1u, figure_save_name,'-dpng','-r400');
    close;
    %%% states %%%
    figure1 = figure('Name',figure_name, 'NumberTitle','off');
    plot(t, x)
    xlabel('time');
    ylabel('x');
    legend('$x_1$', '$x_2$', 'Interpreter','latex');
    figure_save_name = append('figures/xalpha', num2str(alpha(i)), '.png');
    print(figure1, figure_save_name,'-dpng','-r400');
    close;
end
%%%% in one subplot %%%%
%%% Set initial Q %%%
Q = H;
%%% K(t) %%%
for i = 1:length(alpha)
    %%% initial alpha and Q %%%
    Q = alpha(i) * Q;
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    %%% K %%%
    if i == 3
        subplot(2,2,[i, i+1]);
    else
        subplot(2,2,i);
    end
    plot(t_K, K_arr)
    figure_name = append('K(t) (alpha = ', num2str(alpha(i)), ')');
    title(figure_name);
    xlabel('time');
    ylabel('K');
    legend('$K_1$', '$K_2$', '$K_3$', '$K_4$', 'Interpreter','latex');
end
print('figures/SubplotQ3_dK.png','-dpng','-r600');
close;
%%% u(t) %%%
%%% Set initial Q %%%
Q = H;
for i = 1:length(alpha)
    %%% initial alpha and Q %%%
    Q = alpha(i) * Q;
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    %%% K %%%
    if i == 3
        subplot(2,2,[i, i+1]);
    else
        subplot(2,2,i);
    end
    ue = zeros(1);
    K_t	= interp1(t_K, K_arr, t);
    for j = 1:length(t)
        ue(j) = -R_inv*B'*reshape(K_t(j, :), n, n)*x(j,:)';
    end
    plot(t, ue)
    figure_name = append('u(t) (alpha = ', num2str(alpha(i)), ')');
    title(figure_name);
    xlabel('time');
    ylabel('u');
end
print('figures/SubplotQ3_du.png','-dpng','-r600');
close;
%%% states %%%%
%%% Set initial Q %%%
Q = H;
for i = 1:length(alpha)
    %%% initial alpha and Q %%%
    Q = alpha(i) * Q;
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    %%% K %%%
    if i == 3
        subplot(2,2,[i, i+1]);
    else
        subplot(2,2,i);
    end
    ue = zeros(1);
    K_t	= interp1(t_K, K_arr, t);
    for j = 1:length(t)
        ue(j) = -R_inv*B'*reshape(K_t(j, :), n, n)*x(j,:)';
    end
    plot(t, x)
    figure_name = append('System response (alpha = ', num2str(alpha(i)), ')');
    title(figure_name);
    xlabel('time');
    ylabel('x');
    legend('$x_1$', '$x_2$', 'Interpreter','latex');
end
print('figures/SubplotQ3_d.png','-dpng','-r600');
close;
%% Functions %%
%%% Riccati %%%
function d = diff_eq_Riccati(~,k)
global A B Q R_inv n
K	 = reshape(k,n,n);
Kdot = -K * A - A' * K - Q + K * B *  R_inv * B' * K;
d	 = reshape(Kdot, n^2, 1);
end
%%% System %%%
function d = diff_eq_states(t,x)
global A B R_inv n
global t_K K_arr
K_t	= interp1(t_K, K_arr, t);
K	= reshape(K_t, n, n);
u	= -R_inv * B' * K * x;
d	= A * x + B * u;
end