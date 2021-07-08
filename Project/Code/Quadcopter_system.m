%%%%%%%%%%%% Quadcopter system %%%%%%%%%%%%
QuadConstants;
X      = zeros(6, 1);
omega  = ones(6, 1) * 2000;
omega1 = omega(1);
omega2 = omega(2);
omega3 = omega(3);
omega4 = omega(4);
%%%%% problem with omega we shoud use omega^2
x1     = X(1);
x2     = X(2);
x3     = X(3);
x4     = X(4);
x5     = X(5);
x6     = X(6);
a      = zeros(6, 1); % dot(x ) = a(x, u, t) x1 x2 x3 x4 x5 x6
a(1)   = x4 + x5 * sin(x1) * tan(x2) + x6 * cos(x1) * tan(x2);
a(2)   = x5 * cos(x1) - x6 * sin(x1);
a(3)   = (x5 * sin(x1) + x6 * cos(x1)) * sec(x2);
a(4)   = A1 * cos(x2) * sin(x1) + A2 * x5 * x6 + A2 * x5 * x6 + ...
         A3 * (omega2^2 - omega4^2) + ...
         A4 * x5 * (omega1 - omega2 + omega3 - omega4) - x4 / abs(x4) * A5;
a(5)   = B1 * sin(x2) + B3 * (omega1^2 - omega3^2) + ...
         B4 * x4 * (omega1 - omega2 + omega3 - omega4) - x5 / abs(x5) * B5;
a(6)   = C1 * x4 * x5 + C2 * (omega1^2 - omega2^2 + omega3^2 - omega4^2);
%%%%%%%%% Linearization %%%%%%%%%
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