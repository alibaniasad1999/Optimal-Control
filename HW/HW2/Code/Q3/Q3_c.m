%%%% ODE Question 3 part c sub part I %%%%
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
    subplot(2,2,1)
    hold;
for i = 1:length(tf_arr)
    %%% initial final time %%%
    tf = tf_arr(i);
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    %%% solve System equation %%%
    [t, x] = ode45(@diff_eq_states, [0, tf], x0);
    figure_name = strcat('System response tf = ', num2str(tf));
    figure1 = figure('Name',figure_name, 'NumberTitle','off');
    plot(t, x)
    xlabel('time');
    ylabel('x');
    legend('$x_1$', '$x_2$','Interpreter','latex');
    figure_save_name = append('figures/tf', num2str(tf), '.png');
    print(figure1, figure_save_name,'-dpng','-r400');
end
%%%% in one subplot %%%%
for i = 1:length(tf_arr)
    %%% initial final time %%%
    tf = tf_arr(i);
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    %%% solve System equation %%%
    [t, x] = ode45(@diff_eq_states, [0, tf], x0);
    subplot(2,2,i);
    plot(t, x)
    figure_name = strcat('System response tf =  ', num2str(tf));
    title(figure_name);
    xlabel('time');
    ylabel('x');
    legend('$x_1$', '$x_2$','Interpreter','latex');
end
print('figures/SubplotQ3_cI.png','-dpng','-r600');
%%%% ODE Question 3 part c sub part II %%%%
%%% Final time %%%
tf = 3;
for i = 0:2
    H = 10^i * eye(2);
    K0 = H;
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    %%% solve System equation %%%
    [t, x] = ode45(@diff_eq_states, [0, tf], x0);
    figure_name = append('System response H = ', num2str(10^i), 'I');
    figure1 = figure('Name',figure_name, 'NumberTitle','off');
    plot(t, x)
    xlabel('time');
    ylabel('x');
    legend('$x_1$', '$x_2$','Interpreter','latex');
    figure_save_name = append('figures/H', num2str(10^i), '.png');
    print(figure1, figure_save_name,'-dpng','-r400');
end
%%%% in one subplot %%%%
for i = 0:2
    H = 10^i * eye(2);
    K0 = H;
    %%% solve diffrential ricaati %%%
    [t_K, K_arr] = ode45(@diff_eq_Riccati, [tf, 0], K0);
    %%% solve System equation %%%
    [t, x] = ode45(@diff_eq_states, [0, tf], x0);
    subplot(2,2,i+1);
    plot(t, x)
    figure_name = append('System response H = ', num2str(10^i), 'I');
    title(figure_name);
    xlabel('time');
    ylabel('x');
    legend('$x_1$', '$x_2$','Interpreter','latex');
end
print('figures/SubplotQ3_cII.png','-dpng','-r600');
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