%%%%%%%%%%%%%%%%%%%%%% Quadcopter ODE45 %%%%%%%%%%%%%%%%%%%%%%
QuadConstants;
global R_n Q u R_inv
Q         = 10 * eye(6);
u = ones(4, 1) * 2000;
R         = eye(4);
R_inv     = R^-1;
Gamma     = 2 * eye(4);
Gamma_inv = Gamma^-1;
R_n       = (R_inv - Gamma_inv)^-1;
x0	= [0 ;
       1 ;
       0 ;
       0 ;
       0 ;
       0];
tic
[t, x] = ode45(@diff_equ, [0, 3], x0);
x(:, 1:3) = wrapToPi(x(:, 1:3));
toc
plot(t, x)
xlabel('$Time_{\sec}$', 'interpreter', 'latex');
ylabel('$\vec{X}$', 'interpreter', 'latex');
legend('$\phi$', '$\theta$', '$\psi$','$p$','$q$','$r$',...
    'interpreter', 'latex')
print('../Figure/LQR/nonlininear.png','-dpng','-r500')
function d = diff_equ(t, X)
global u Q R_n R_inv
x = X(1:6);
[A, B] = Quadcopter_system(x, u);
x(1:3) = wrapToPi(x(1:3));
t
try
    [k, ~, ~] = icare(A, B, Q, R_n);
    u = -R_inv*B'*k*x;
    d = A * x + B * u;
catch
    k = zeros(4, 6);
    u = -R_inv*B'*k*x;
    d = A * x + B * u;
end
end
