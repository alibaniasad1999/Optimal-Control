%%%% ODE Question 3 part a %%%%
global A B R n R_inv K1 K2
A = [ 0    1  ;
     -1 -0.1] ;
B = [ 0    1]';
R = 1;
R_inv = R^-1;
x0	= [1 1]';
n = 2;
%%%% K %%%%
K1 = ...
[ -2.0175125161240487181235605589055,  0.4142135623730950488016887242097 ;
   0.4142135623730950488016887242097, -1.4558861031613938986501027136414];
K2 = ...
[ 1.8175125161240487181235605589055, 0.4142135623730950488016887242097 ;
  0.4142135623730950488016887242097, 1.2558861031613938986501027136414];
[t_1,x_1] = ode45(@diff_eq_statesK1,[0, 10], x0);
[t_2,x_2] = ode45(@diff_eq_statesK2,[0, 10], x0);
figure1 = figure('Name','System simulation with K1','NumberTitle','off');
plot(t_1, x_1)
xlabel('time');
ylabel('x');
legend('$x_1$', '$x_2$','Interpreter','latex');
print(figure1, 'figures/K1ODE.png','-dpng','-r400');
figure2 = figure('Name','System simulation with K2','NumberTitle','off');
plot(t_2, x_2)
xlabel('time');
ylabel('x');
legend('$x_1$', '$x_2$','Interpreter','latex');
print(figure2, 'figures/K2ODE.png','-dpng','-r400');
%%%%%%%%%%% u %%%%%%%%%%%
ue = zeros(1);
for i = 1:length(t_2)
    ue(i) = -R_inv*B'*K2*x_2(i,:)';
end
figure3 = figure('Name','u(t)','NumberTitle','off');
plot(t_2, ue)
xlabel('time');
ylabel('u');
print(figure3, 'figures/uODE.png','-dpng','-r400');
%% Functions for K1 and K2%%
function d = diff_eq_statesK1(~, x)
global A B R_inv K1
u	= -R_inv*B'*K1*x;	
d	= A*x + B*u;
end
function d = diff_eq_statesK2(~, x)
global A B R_inv K2
u	= -R_inv*B'*K2*x;	
d	= A*x + B*u;
end
