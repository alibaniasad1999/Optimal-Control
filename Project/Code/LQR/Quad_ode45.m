%%%%%%%%%%%%%%%%%%%%%% Quadcopter ODE45 %%%%%%%%%%%%%%%%%%%%%%
QuadConstants;
global A B k R
[A, B] = Quadcopter_system(zeros(6, 1), ones(4, 1) * 2000);
Q = 10 * eye(6);
R = eye(4);
[k, ~, ~] = icare(A, B, Q, R);
[t, x] = ode45(@diff_equ, [0, 10], [1; 1 ; 1; zeros(3, 1)+1]);
x(:, 1:3) = wrapToPi(x(:, 1:3));
plot(t, x)
legend('$\phi$', '$\theta$', '$\psi$','$p$','$q$','$r$',...
    'interpreter', 'latex')
xlabel('$Time_{(\sec)}$', 'interpreter', 'latex');
ylabel('$System~State$', 'interpreter', 'latex');
print('../../Figure/LQR/LQRall.png','-dpng','-r500')
function d = diff_equ(~, X)
x = X(1:6);
x(1:3) = wrapToPi(x(1:3));
global A B k R
d = (A - B * R^-1 * B' * k) * x;
end
