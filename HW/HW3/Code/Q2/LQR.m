clc
global A B Q R n R_inv
A	= [0	1
    -1 -0.1];
B	= [0
    1];
Q	= 1*eye(2);
R	= 1;
R_inv = 1;
H	= 1*eye(2);
tf	= 3;

K0	= H;
n	= 2;
k0	= reshape(K0,n^2,1);

global t_K K_arr
[t_K,K_arr] = ode45(@diff_eq_Riccati,[tf,0],k0);

% figure(100)
% hold on
% plot(t_K,K_arr)
x0	= [1 1]';
[t,x] = ode45(@diff_eq_states,[0,tf],x0);
figure(101)
hold on
plot(t,x)
xlabel('$Time_{\sec}$', 'interpreter', 'latex');
ylabel('$\vec{X}$', 'interpreter', 'latex');
legend('$x_1$', '$x_2$', 'interpreter', 'latex');
u_max =  0.4;
u_min = -0.4;
u = zeros(length(t), 1);

for i=1:length(t)
    K_t	= interp1(t_K, K_arr, t(i));
    K	= reshape(K_t,n,n);
    u(i)	= -R_inv*B'*K*x(i,:)';
    if u(i)>u_max
        u(i) = u_max;
    elseif u(i)<u_min
        u(i) = u_min;
    end
end
figure(102)
hold on
plot(t,u)
xlabel('$Time_{\sec}$', 'interpreter', 'latex');
ylabel('$u$', 'interpreter', 'latex');
print(101, '../../Figure/Q2/LQR.png','-dpng','-r300')
print(102, '../../Figure/Q2/conrtol LQR.png','-dpng','-r300')


%
% % t_a = 0:0.01:10;
% t_a		= t_n;
% x_a	= x0*exp(c*t_a);
%
%
% figure
% plot(t_n,x_n);
% hold on
% plot(t_a, x_a)
% legend('Numerical','Anlytical')
%%

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

u_max = 0.4;
u_min = -0.4;


if u>u_max
    u = u_max;
elseif u<u_min
    u = u_min;
end
d	= A*x + B*u;
end

