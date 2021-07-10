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
[t, x] = ode45(@diff_equ, [0, 1], x0);
x(:, 1:3) = wrapToPi(x(:, 1:3));
plot(t, x)
legend('$\phi$', '$\theta$', '$\psi$','$p$','$q$','$r$',...
    'interpreter', 'latex') 
function d = diff_equ(t, X)
global u Q R
if t == 0
    u = ones(4, 1) * 2000;
end
x = X(1:6);
[A, B] = Quadcopter_system(x, u);
[~, k1, ~] = icare(A, B, Q, R);
x(1:3) = wrapToPi(x(1:3));
d = (A - B * k1) * x;
u = -k1 * x;
end
