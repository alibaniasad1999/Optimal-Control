global A B Q n R_inv
[A, B] = Quadcopter_system(zeros(6, 1), ones(4, 1) * 2000);
Q	= 10*eye(6);
R	= eye(4);
R_inv = R^-1;
H	= 10*eye(6);
tf	= 10;

K0	= H;
n	= 6;
k0	= reshape(K0,n^2,1);
global t_K K_arr
[t_K,K_arr] = ode45(@diff_eq_Riccati,[tf,0],k0);
x0	= [1.7 ;
       1.6 ;
       1.5 ;
       1.4 ;
       1.3 ;
       1.2];
[t,x] = ode45(@diff_eq_states,[0,tf],x0);
figure(101)
plot(t, x)
legend('$\phi$', '$\theta$', '$\psi$','$p$','$q$','$r$',...
    'interpreter', 'latex') 
xlabel('$Time_{(\sec)}$', 'interpreter', 'latex');
ylabel('$System~State$', 'interpreter', 'latex');
%% Functions %%
function d = diff_eq_Riccati(~,k)
global A B Q R_inv n
K	= reshape(k,n,n);
Kdot	= -K*A - A'*K - Q + K*B*R_inv*B'*K;
d	= reshape(Kdot,n^2,1);
end

function d = diff_eq_states(t,x)
global A B R_inv n
global t_K K_arr
K_t	= interp1(t_K, K_arr, t);
K	= reshape(K_t,n,n);
u	= -R_inv*B'*K*x;
d	= A*x + B*u;
end

