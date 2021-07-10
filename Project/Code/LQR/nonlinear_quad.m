%%%%%%%%%%%%%%%%%%%%%% Quadcopter ODE45 %%%%%%%%%%%%%%%%%%%%%%
QuadConstants;
global R Q
Q = 10 * eye(6);
R = eye(4);
x0	= [0 ;
    1 ;
    0 ;
    0 ;
    0 ;
    0];
tic
[t, x] = ode45(@diff_equ, [0, 7], x0);
x(:, 1:3) = wrapToPi(x(:, 1:3));
toc
plot(t, x)
xlabel('$Time_{\sec}$', 'interpreter', 'latex');
ylabel('$\vec{X}$', 'interpreter', 'latex');
legend('$\phi$', '$\theta$', '$\psi$','$p$','$q$','$r$',...
    'interpreter', 'latex')
print('../Figure/LQR/nonlininear.png','-dpng','-r500')
function d = diff_equ(t, X)
global u Q R
if t == 0
    u = ones(4, 1) * 2000;
end
x = X(1:6);
[A, B] = Quadcopter_system(x, u);
x(1:3) = wrapToPi(x(1:3));
t
try
    [~, k1, ~] = icare(A, B, Q, R);
    d = (A - B * k1) * x;
    u = -k1 * x;
catch
    k1 = zeros(4, 6);
    d = (A - B * k1) * x;
    u = -k1 * x;
end
end
