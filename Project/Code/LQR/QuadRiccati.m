global A B Q R n R_inv
QuadConstants;
A = [0  0  0 1 0 0  ;
     0  0  0 0 1 0  ;
     0  0  0 0 0 1  ;
     A1 0  0 0 0 0  ;
     0  B1 0 0 0 0  ;
     0  0  0 0 0 0] ;
B = [0   0   0   0  ;
     0   0   0   0  ;
     0   0   0   0  ;
     0   A3  0  -A3 ;
     B3  0  -B3  0  ;
     C2 -C2  C2  C2];
Q	= 1000*eye(6);
R	= 1;
R_inv = 1;
H	= 10*eye(6);
tf	= 100;

K0	= H;
n	= 6;
k0	= reshape(K0,n^2,1);
global t_K K_arr
[t_K,K_arr] = ode45(@diff_eq_Riccati,[tf,0],k0);
x0	= ones(6, 1)*0.1;
[t,x] = ode45(@diff_eq_states,[0,tf],x0);
% x(:, 1:3) = wrapToPi(x(:, 1:3));
plot(t, x)
figure(101)
plot(t, x)
legend('$\phi$', '$\theta$', '$\psi$','$p$','$q$','$r$',...
    'interpreter', 'latex') 
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

