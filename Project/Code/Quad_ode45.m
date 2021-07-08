%%%%%%%%%%%%%%%%%%%%%% Quadcopter ODE45 %%%%%%%%%%%%%%%%%%%%%%
QuadConstants;
global A B k R
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
Q = 10 * eye(6);
R = eye(4);
[k, ~, ~] = icare(A, B, Q, R);
[t, x] = ode45(@diff_equ, [0, 10], [1; 1 ; 1; zeros(3, 1)+1]);
x(:, 1:3) = wrapToPi(x(:, 1:3));
plot(t, x)
legend('$\phi$', '$\theta$', '$\psi$','$p$','$q$','$r$',...
    'interpreter', 'latex') 
function d = diff_equ(~, X)
x = X(1:6);
x(1:3) = wrapToPi(x(1:3));
global A B k R
d = (A - B * R^-1 * B' * k) * x;
end
