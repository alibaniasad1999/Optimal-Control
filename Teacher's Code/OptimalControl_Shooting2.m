%--------------------------------------------------------------------------
% 1395/3/8 (May, 28, 2016)
% by Nima Assadian
% Department of Aerospace Engineering
% Sharif University of Technology
%--------------------------------------------------------------------------
% This program is designed for optimal control course. This course is
% intended for students of engineering.
%--------------------------------------------------------------------------
% In this code, an optimal control problem of a linear system
% with quadratic cost function is solved throught the simple shooting method.
%--------------------------------------------------------------------------
%==========================================================================
% Main Routine
%==========================================================================
function OptimalControl_Shooting2()
% clc

global tf R Q H A B x0 inv_R

% ------------------        Dynamic System Modeling       -----------------
% Dynamic System Modeling is an approach to understanding the behaviour of
% systems. A linear control system can be written as:xdot=Ax+Bu
% where x in R(n) is called the state, u in R(m) is the input and
% A and B are matrices with proper dimensions, either constant or
% time-varying.
% Following is a brief description of the system modeling's input arguments:
% A    :  state matrix of the given system
% B    :  input matrix of the given system
% x0   :  initial state vector

A	= [0	1
	0 0];
B	= [0
	1];
x0		= [1 1]';
% -------------------      Cost Function         --------------------------
%  A control problem includes a cost functional that is a function of state
%  and control variables.In optimal control theory, the following objective
%  function is minimized:
% J(u, x(t), t) =h(x(tf),tf)+int(g(x(t),u(t),t)dt
% A special case of the general nonlinear optimal control problem given in
% the previous section is the linear quadratic regulator(LQR) optimal
% control problem. The LQR problem is stated as follows.
% Minimize the quadratic continuous-time cost functional
% J(u, x(t), t) = 0.5 * x(tf) H x(tf)+ 0.5 * int(x(t)Q(t)x(t) + u(t)R(t)u(t))dt
% where Q(t) and H are symmetric positive semi-definite n × n matrices,
% R(t) is a symmetric positive definite m×m matrix.
% Note that the LQR cost functional can be thought of physically as
% attempting to minimize the control energy (measured as a quadratic form).
% Following is a brief description of the cost function's input arguments:
% x(t)Q(t)x(t)  :  penalizes the transient state deviation
% x(tf) H x(tf) :  penalizes the finite state
% u(t)R(t)u(t)  :  penalizes the control effort
% H             :  the finite terminal weighting matrix
% Q             :  the state weighting matrix
% R             :  the control weighting matrix
% tf            :  the final time
H		= 1000*eye(2);
Q		= 1*eye(2);
R		= 1;
inv_R   = inv(R);
tf		= 2;
%==========================================================================
%  Simple shooting loop
%==========================================================================
tol			= 1e-8;
norm_F      = tol + 1;
max_count	= 100;
counter		= 0;
p0 = [-0.4 1]';
phi0	= eye(4);
phi0	= reshape(phi0, 16, 1);

options = odeset('AbsTol', 1e-10, 'RelTol', 1e-10);
while (norm_F > tol && counter < max_count)
	counter = counter + 1;
	%======================================================================
	% computing the F
	%======================================================================
	z0 = [x0;p0;phi0];
	[time_x, z] = ode45(@diff_equ, [0 tf], [z0], options);
	xf	= z(end,1:2)';
	pf	= z(end,3:4)';
	phi	= z(end,5:end);
	phi	= reshape(phi,4,4);
% 	F	= pf - H*xf;
	F	= xf - [0;0];
	norm_F	= norm(F,2);
	dF_dy	= phi(1:2,3:4);
% 	p0		= p0 - inv(dF_dy)*F;
	p0		= p0 - dF_dy\F;
end
figure;
u = -inv_R*B'*z(:,3:4)';
plot(time_x,[z(:,1:2),u'])
legend('x1','x2','u')

end


%==========================================================================
% FUNCTIONS
%==========================================================================
% This function computes model differential equations, that is expressed as:
% xdot=Ax+Bu
% Note: The initial condition is initial state.
function d = diff_equ(t, Z)
global A B Q inv_R
X	= Z(1:2);
P	= Z(3:4);
phi	= Z(5:end);
phi	= reshape(phi,4,4);

d		= A*X - B*inv_R*B'*P;
d(3:4)	= -Q*X - A'*P;

Ap	= [  A	-B*inv_R*B'
		-Q	-A'];
dphi	= Ap*phi;
dphi	= reshape(dphi,16,1);
d	= [d;dphi];
end

%==========================================================================
