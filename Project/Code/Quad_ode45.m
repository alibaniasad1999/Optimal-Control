%%%%%%%%%%%%%%%%%%%%%% Quadcopter ODE45 %%%%%%%%%%%%%%%%%%%%%%
global A B
A = [0  0  0 1 0 0  ;
     0  0  0 0 1 0  ;
     0  0  0 0 0 1  ;
     A1 0  0 0 0 0  ;
     0  B1 0 0 0 0  ;
     0  0  0 0 0 0] ;
[t, x] = ode45(@diff_equ, [0, 10], zeros(6, 1));
plot(t, x)
function d = diff_equ(~, X)
x = X(1:6);
global A
d = A*x;
end
