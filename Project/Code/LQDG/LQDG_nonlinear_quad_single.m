% Distance between rotor center to quad center (m)
d_cg = 0.295;
% Distance between D point to quad center (m)
h_cg = 0.0078;
% Thrust factor (non dimensional)
b = 1.10794e-5;
% Rotor torque drag factor (non dimensional)
d = 3.2e-6;
% Total mass (kg)
m_tot = 1.774;
% Quad Moment of inertia respect to axix X Body (kg.m^2)
J_xx = 0.0297598;
% Quad Moment of inertia respect to axix Y Body (kg.m^2)
J_yy = 0.0298565;
% Quad Moment of inertia respect to axix Y Body (kg.m^2)
J_zz = 0.058510;
% Quad blade Moment (kg.m^2)
J_R = 4.439e-5;
% Mass on X axis bearing (kg)
m1 = 1.272;
% Mass on Y axis bearing (kg)
m2 = 1.074;
% Mass on Z axis bearing (kg)
m3 = 1.693;
% Bearing radius in X axix (m)
r_x = 0.01;
% Bearing radius in Y axix (m)
r_y = 0.01;
% Bearing radius in Z axix (m)
r_z = 0.025;
% Static bearing friction (non dimensional)
miu_s = 0.003;
% Dynamic bearing friction (non dimensional)
miu_k = 0.002;
% Motor constant (non dimensional)
tau = 0.15;
% Delay of motor (sec)
T_d = 0.02;
% Gravity force (m/s^2)
g = 9.81;
% A Constants
A1 = (h_cg * g * m_tot) / (m_tot * h_cg^2 + J_xx);
A2 = (2*m_tot * h_cg^2 + J_yy -J_zz) / (m_tot * h_cg^2 + J_xx);
A3 = (b * d_cg) / (m_tot * h_cg^2 + J_xx);
A4 = (J_R) / (m_tot * h_cg^2 + J_xx);
A5 = (m1 * g * miu_k * r_x) / (m_tot * h_cg^2 + J_xx);
% B Constants
B1 = (h_cg * g * m_tot) / (m_tot * h_cg^2 + J_yy);
B2 = (-2*m_tot * h_cg^2 - J_xx + J_zz) / (m_tot * h_cg^2 + J_yy);
B3 = (b * d_cg) / (m_tot * h_cg^2 + J_yy);
B4 = (-J_R) / (m_tot * h_cg^2 + J_yy);
B5 = (m2 * g * miu_k * r_y) / (m_tot * h_cg^2 + J_yy);
% C Constants
C1 = (J_xx - J_yy) / (J_zz);
C2 = (d) / (J_zz);
C3 = (m3 * g * miu_k * r_z) / (J_zz);
global R_n Q u
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
global u Q R_n
x = X(1:6);
[A, B] = Quadcopter_system(x, u);
x(1:3) = wrapToPi(x(1:3));
t
try
    [~, k1, ~] = icare(A, B, Q, R_n);
    d = (A - B * k1) * x;
    u = -k1 * x;
catch
    k1 = zeros(4, 6);
    d = (A - B * k1) * x;
    u = -k1 * x;
end
end
%%%%%%%%%%%% Quadcopter system %%%%%%%%%%%%
function [A, B] = Quadcopter_system(X, omega)
QuadConstants;
%X      = zeros(6, 1);
%u      = ones(4, 1) * 2000^2;
% omega  = sqrt(u);
omega1 = omega(1);
omega2 = omega(2);
omega3 = omega(3);
omega4 = omega(4);
%%%%% problem with omega we shoud use omega^2
x1     = X(1);
x2     = X(2);
% x3     = X(3);
x4     = X(4);
x5     = X(5);
x6     = X(6);
% a      = zeros(6, 1); % dot(x ) = a(x, u, t) x1 x2 x3 x4 x5 x6
% a(1)   = x4 + x5 * sin(x1) * tan(x2) + x6 * cos(x1) * tan(x2);
% a(2)   = x5 * cos(x1) - x6 * sin(x1);
% a(3)   = (x5 * sin(x1) + x6 * cos(x1)) * sec(x2);
% a(4)   = A1 * cos(x2) * sin(x1) + A2 * x5 * x6 + A2 * x5 * x6 + ...
%          A3 * (omega2^2 - omega4^2) + ...
%          A4 * x5 * (omega1 - omega2 + omega3 - omega4) - x4 / abs(x4) * A5;
% a(5)   = B1 * sin(x2) + B3 * (omega1^2 - omega3^2) + ...
%          B4 * x4 * (omega1 - omega2 + omega3 - omega4) - x5 / abs(x5) * B5;
% a(6)   = C1 * x4 * x5 + C2 * (omega1^2 - omega2^2 + omega3^2 - omega4^2);
%%%%%%%%% Linearization %%%%%%%%%
%%%%% A matrix %%%%%
A11    = x5 * cos(x1) * tan(x2)   - x6 * sin(x1) * tan(x2); %da1dx1
A12    = x5 * sin(x1) / cos(x2)^2 + x6 * cos(x1) / cos(x2)^2; %da1dx2
A13    = 0; %da1dx3
A14    = 1; %da1dx4
A15    = sin(x1) * tan(x2); %da1dx5
A16    = cos(x1) * tan(x2); %da1dx6
A21    = -x5 * sin(x1) -x6 * cos(x1); %da2dx1
A22    = 0; %da2dx2
A23    = 0; %da2dx3
A24    = 0; %da2dx4
A25    =  cos(x1); %da2dx5
A26    = -sin(x1); %da2dx6
A31    = (x5 * cos(x1) - x6 * sin(x1)) * sec(x2);
A32    = (x5 * sin(x1) + x6 * cos(x1)) * sec(x2) * tan(x2);
A33    = 0;
A34    = 0;
A35    = sin(x1) * sec(x2);
A36    = cos(x1) * sec(x2);
A41    =  A1 * cos(x2) * cos(x1);
A42    = -A2 * sin(x1) * sin(x2);
A43    = 0;
A44    = 0;
A45    = A2 * x6 + A4 * (omega1 - omega2 + omega3 - omega4);
A46    = 0;
A51    = 0;
A52    = B1 * cos(x2);
A53    = 0;
A54    = B2 * x6 + B4 * (omega1 - omega2 + omega3 - omega4);
A55    = 0;
A56    = B2 * x6;
A61    = 0;
A62    = 0;
A63    = 0;
A64    = C1 * x5;
A65    = C1 * x4;
A66    = 0;
%%%%% B matrix %%%%%
B11    = 0;
B12    = 0;
B13    = 0;
B14    = 0;
B21    = 0;
B22    = 0;
B23    = 0;
B24    = 0;
B31    = 0;
B32    = 0;
B33    = 0;
B34    = 0;
%%% about omega^2 %%%
% B41    = A4 * x5 / (2 * omega1);
% B42    = A3 - A4 * x5 / (2 * omega2);
% B43    = A4 * x5 / (2 * omega3);
% B44    = A3 - A4 * x5 / (2 * omega4);
% B51    = B3 + B4 * x4 / (2 * omega1);
% B52    = -B4 * x4 / (2 * omega2);
% B53    = -B3 + B4 * x4 / (2 * omega3);
% B54    = -B4 * x4 / (2 * omega4);
% B61    =  C2;
% B62    = -C2;
% B63    =  C2;
% B64    = -C2;
%%% about omega %%%
B41    = A4 * x5;
B42    = 2 * A3 * omega2 - A4 * x5;
B43    = A4 * x5;
B44    = -2 * A3 * omega4 - A4 * x5;
B51    =  2 * B3 * omega1 + B4 * x4;
B52    = -B4 * x4;
B53    = -2 * B3 * omega3 + B4 * x4;
B54    = -B4 * x4;
B61    =   2 * C2 * omega1;
B62    =  -2 * C2 * omega2;
B63    =   2 * C2 * omega3;
B64    =  -2 * C2 * omega4;
%%%%%%%%%%%%%% Ax Bu %%%%%%%%%%%%%%
A      = [A11 A12 A13 A14 A15 A16 ;
          A21 A22 A23 A24 A25 A26 ;
          A31 A32 A33 A34 A35 A36 ;
          A41 A42 A43 A44 A45 A46 ;
          A51 A52 A53 A54 A55 A56 ;
          A61 A62 A63 A64 A65 A66];
 B     = [B11 B12 B13 B14 ;
          B21 B22 B23 B24 ;
          B31 B32 B33 B34 ;
          B41 B42 B43 B44 ;
          B51 B52 B53 B54 ;
          B61 B62 B63 B64];
end