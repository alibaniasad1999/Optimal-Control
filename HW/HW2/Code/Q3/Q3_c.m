%% ODE Question 3 part c sub part I %%
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
%%%% initial condition %%%%
x0	= [1 1]';
K0 = H;
%%%% ODE %%%%
global t_K K_arr
tf_arr = [3 5 10 15];
for i = 1:length(tf_arr)
    %%% initial final time %%%
    tf = tf_arr(i);
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    %%% solve System equation %%%
    [t, x] = ode45(@diff_eq_states, [0, tf], x0);
    %%% K %%%
    figure_name = strcat('System response tf = ', num2str(tf));
    figure1K = figure('Name',figure_name, 'NumberTitle','off');
    plot(t_K, K_arr);
    xlabel('time');
    ylabel('K');
    legend('$K_1$', '$K_2$', '$K_3$', '$K_4$', 'Interpreter','latex');
    figure_save_name = append('figures/K', num2str(tf), '.png');
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
    figure_save_name = append('figures/u', num2str(tf), '.png');
    print(figure1u, figure_save_name,'-dpng','-r400');
    close;
    %%% states %%%
    figure1 = figure('Name',figure_name, 'NumberTitle','off');
    plot(t, x)
    xlabel('time');
    ylabel('x');
    legend('$x_1$', '$x_2$', 'Interpreter','latex');
    figure_save_name = append('figures/tf', num2str(tf), '.png');
    print(figure1, figure_save_name,'-dpng','-r400');
    close;
end
%%%% in one subplot %%%%
%%% K(t) %%%%
for i = 1:length(tf_arr)
    %%% initial final time %%%
    tf = tf_arr(i);
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    %%% solve System equation %%%
    [t, x] = ode45(@diff_eq_states, [0, tf], x0);
    subplot(2,2,i);
    plot(t_K, K_arr);
    figure_name = strcat('K(t) (tf =  ', num2str(tf), ')');
    title(figure_name);
    xlabel('time');
    ylabel('K');
    legend('$K_1$', '$K_2$', '$K_3$', '$K_4$', 'Interpreter','latex');
end
print('figures/SubplotQ3_cKI.png','-dpng','-r600');
close;
%%% u(t) %%%%
for i = 1:length(tf_arr)
    %%% initial final time %%%
    tf = tf_arr(i);
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    %%% solve System equation %%%
    [t, x] = ode45(@diff_eq_states, [0, tf], x0);
    subplot(2,2,i);
    ue = zeros(1);
    K_t	= interp1(t_K, K_arr, t);
    for j = 1:length(t)
        ue(j) = -R_inv*B'*reshape(K_t(j, :), n, n)*x(j,:)';
    end
    plot(t, ue);
    figure_name = strcat('u(t) (tf =  ', num2str(tf), ')');
    title(figure_name);
    xlabel('time');
    ylabel('u');
end
print('figures/SubplotQ3_cuI.png','-dpng','-r600');
close;
%%% states %%%%
for i = 1:length(tf_arr)
    %%% initial final time %%%
    tf = tf_arr(i);
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    %%% solve System equation %%%
    [t, x] = ode45(@diff_eq_states, [0, tf], x0);
    subplot(2,2,i);
    plot(t, x)
    figure_name = strcat('System response (tf =  ', num2str(tf), ')'));
    title(figure_name);
    xlabel('time');
    ylabel('x');
    legend('$x_1$', '$x_2$','Interpreter','latex');
end
print('figures/SubplotQ3_cI.png','-dpng','-r600');
close;
%% ODE Question 3 part c sub part II %%
%%% Final time %%%
tf = 3;
for i = 0:2
    H = 10^i * eye(2);
    K0 = H;
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    %%% solve System equation %%%
    [t, x] = ode45(@diff_eq_states, [0, tf], x0);
    %%% K %%%
    figure_name = append('System response H = ', num2str(10^i), 'I');
    figure1K = figure('Name',figure_name, 'NumberTitle','off');
    plot(t_K, K_arr);
    xlabel('time');
    ylabel('K');
    legend('$K_1$', '$K_2$', '$K_3$', '$K_4$', 'Interpreter','latex');
    figure_save_name = append('figures/KH', num2str(10^i), '.png');
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
    figure_save_name = append('figures/uH', num2str(10^i), '.png');
    print(figure1u, figure_save_name,'-dpng','-r400');
    close;
    %%% states %%%
    figure1 = figure('Name',figure_name, 'NumberTitle','off');
    plot(t, x)
    xlabel('time');
    ylabel('x');
    legend('$x_1$', '$x_2$', 'Interpreter','latex');
    figure_save_name = append('figures/xH', num2str(10^i), '.png');
    print(figure1, figure_save_name,'-dpng','-r400');
    close;
end
%%%% in one subplot %%%%
%%% K(t) %%%
for i = 0:2
    H = 10^i * eye(2);
    K0 = H;
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    subplot(2,2,i+1);
    plot(t_K, K_arr)
    figure_name = append('K(t) (H = ', num2str(10^i), 'I)');
    title(figure_name);
    xlabel('time');
    ylabel('K');
    legend('$K_1$', '$K_2$', '$K_3$', '$K_4$', 'Interpreter','latex');
end
print('figures/SubplotQ3_cKII.png','-dpng','-r600');
close;
%%% u(t) %%%
for i = 0:2
    H = 10^i * eye(2);
    K0 = H;
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    %%% solve System equation %%%
    [t, x] = ode45(@diff_eq_states, [0, tf], x0);
    subplot(2,2,i+1);
    ue = zeros(1);
    K_t	= interp1(t_K, K_arr, t);
    for j = 1:length(t)
        ue(j) = -R_inv*B'*reshape(K_t(j, :), n, n)*x(j,:)';
    end
    plot(t, ue);
    figure_name = append('u(t) (H = ', num2str(10^i), 'I)');
    title(figure_name);
    xlabel('time');
    ylabel('u');
end
print('figures/SubplotQ3_cuII.png','-dpng','-r600');
close;
%%% states %%%%
for i = 0:2
    H = 10^i * eye(2);
    K0 = H;
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    %%% solve System equation %%%
    [t, x] = ode45(@diff_eq_states, [0, tf], x0);
    subplot(2,2,i+1);
    plot(t, x)
    figure_name = append('System response (H = ', num2str(10^i), 'I)');
    title(figure_name);
    xlabel('time');
    ylabel('x');
    legend('$x_1$', '$x_2$','Interpreter','latex');
end
print('figures/SubplotQ3_cII.png','-dpng','-r600');
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