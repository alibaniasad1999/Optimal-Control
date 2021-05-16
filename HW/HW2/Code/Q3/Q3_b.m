%%%% ODE Question 3 part b %%%%
global A B Q R R_inv K
A = [ 0    1  ;
     -1 -0.1] ;
B = [ 0    1]';
Q = [ 1    0  ;
      0    1] ;
%%%% Generate logarithmically spaced vector %%%%
R = logspace(-1,3);
%%%% Costs %%%%%
uCost = zeros(1);
xCost = zeros(1);
%%%% initial condition %%%%
x0	= [1 1]';
%%%% loop for beta %%%%
for i = 1:length(R)
    % Solve reccati %
    [X1,K1,L1] = icare(A, B, Q, R(i));
    K = X1;
    R_inv = R(i)^-1;
    [t,x] = ode45(@diff_eq_states,[0, 200], x0);
    xCost(i) = trapz(t, (x(:, 1).^2+x(:, 2).^2));
    ue = zeros(1);
    for j = 1:length(t)
        ue(j) = -R_inv*B'*K*x(j,:)';
    end
    uCost(i) = trapz(t, (ue.^2));
end
%%%% Ploter %%%%
%%% Final System %%%
%%% X(x_1^2 + x_2^2) %%%
figure1 = figure('Name','Final System with biggest beta','NumberTitle','off');
plot(t, x)
xlabel('time');
ylabel('x');
legend('x1', 'x2');
print(figure1, 'figures/FinalBeta.png','-dpng','-r400');
%%% X(x_1^2 + x_2^2) %%%
figure2 = figure('Name','X cost','NumberTitle','off');
loglog(R, xCost)
xlabel('$\beta$','Interpreter','latex','FontSize',20);
ylabel('Cost');
grid on
print(figure2, 'figures/xCost.png','-dpng','-r400');
%%% u %%%%
figure3 = figure('Name','u cost','NumberTitle','off');
loglog(R, uCost)
xlabel('$\beta$','Interpreter','latex','FontSize',20);
ylabel('Cost');
grid on
print(figure3, 'figures/uCost.png','-dpng','-r400');
%% System %%
function d = diff_eq_states(~, x)
global A B R_inv K
u	= -R_inv * B' * K * x;	
d	=  A * x + B * u;
end
