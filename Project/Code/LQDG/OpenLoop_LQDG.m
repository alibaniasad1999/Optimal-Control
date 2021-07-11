global A B Q n S1 S2 R1_inv
% QuadConstants;
% A = [0  0  0 1 0 0  ;
%      0  0  0 0 1 0  ;
%      0  0  0 0 0 1  ;
%      A1 0  0 0 0 0  ;
%      0  B1 0 0 0 0  ;
%      0  0  0 0 0 0] ;
% B = [0   0   0   0  ;
%      0   0   0   0  ;
%      0   0   0   0  ;
%      0   A3  0  -A3 ;
%      B3  0  -B3  0  ;
%      C2 -C2  C2  C2];
%  
[A, B] = Quadcopter_system(zeros(6, 1), ones(4, 1) * 2000);
Q      = 100*eye(6);
R1     = eye(4);
R1_inv = R1^-1;
R2     = 2 * eye(4);
R2_inv = R2^-1;
S1     = B* R1_inv * B';
S2     = B* R2_inv * B';
H	   = 100*eye(6);
tf     = 20;
p0	= [H;H];
n	= 6;
p0	= reshape(p0,2 * n^2,1);
global t_p p_arr
[t_p,p_arr] = ode45(@diff_eq_Riccati,[tf,0],p0);
x0	= [1 ;
       1 ;
       1 ;
       1 ;
       1 ;
       1];
   
[t,x] = ode45(@diff_eq_states,[0,tf],x0);
% x(:, 1:3) = wrapToPi(x(:, 1:3));
figure(101)
plot(t, x)
legend('$\phi$', '$\theta$', '$\psi$','$p$','$q$','$r$',...
    'interpreter', 'latex') 
%% Functions %%
function d = diff_eq_Riccati(~,p)
global A Q n S1 S2
p	= reshape(p,2 * n,n);
p1 = p(1:n, :);
p2 = p(n+1: end, :);
p1dot	= -p1*A - A'*p1 - Q +  p1*S1*p1 + p1 * S2 * p2;
p2dot	= -p2*A - A'*p2 - Q +  p2*S2*p2 + p2 * S1 * p1;
d	= reshape([p1dot;p2dot],2 * n^2,1);
end

function d = diff_eq_states(t,x)
global n u R1_inv
global t_p p_arr S1 S2
if t == 0
    u = zeros(4, 1);
end
[A, B] = Quadcopter_system(x, u);
p_t	= interp1(t_p, p_arr, t);
p	= reshape(p_t,2 * n,n);
p1 = p(1:n, :);
p2 = p(n+1: end, :);
d	= (A - S1 * p1 - S2 * p2) * x;
u = R1_inv * B' * p1 * x;
end

