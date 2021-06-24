%--------------------------------------------------------------------------
% 1392/2/10 (April, 30th, 2013)
% by Nima Assadian and Alireza Sharifi
% Department of Aerospace Engineering
% Sharif University of Technology
%--------------------------------------------------------------------------
% This program is designed for optimal control course. This course is
% intended for students of engineering.
%--------------------------------------------------------------------------
%==========================================================================
% Main Routine
%==========================================================================
function my_OptimalControl_Optimization()
clc

global tf t_u R Q H A B x0 dt plot_flag

% ------------------        Dynamic System Modeling       -----------------
% Dynamic System Modeling is an approach to understanding the behaviour of
% systems. A linear control system can be written as:xdot=Ax+Bu and y=Cx;
% where x?R(n)is called the state,u?R(m)the input and y?R(p)the output,and
% A,B and C are matrices with proper dimensions, either constant or
% time-varying.
% Following is a brief description of the system modeling's input arguments:
% A    :  state matrix of the given system
% B    :  input matrix of the given system
% x0   :  initial state vector

A	= [-0.7	3
    -0.1 -1];
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
% where Q(t) and H are symmetric positive semi-definite n � n matrices,
% R(t) is a symmetric positive definite m�m matrix.
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
H		= 1*eye(2);
Q		= 10*eye(2);
R		= 1;
tf		= 3;
% -------------------      Discretization         -------------------------
% Numerical Solution of Optimal Control Problems by Direct Method by an
% appropriate discretization of control and state variables.
% Choose an integer N, a step size dt and grid points t_u as follows:
% dt=tf/N
% t_u = i*dt   (i = 0,1,...,N)
% Approximation of control and state at grid points as follows:
% u(ti) == ui  (i = 0,1,...,N)
% x(ti) == xi  (i = 0,1,...,N)
% Optimization variable for discretization as follows:
% U= (u(0),u(1),...;u(N-1),u(N))

N		= 100;
dt		= tf/N;
U		= zeros(N+1, 1);
t_u		= 0:dt:tf;
% -------------------      Execution Options         ----------------------
plot_flag	= 0;

%==========================================================================
%  Optimization Loop
%==========================================================================
tol				= 1e-8;
norm_gradient	= tol + 1;
max_count		= 200;
counter			= 0;
while (norm_gradient > tol && counter < max_count)
    counter = counter + 1;
    %
    %======================================================================
    % Gradient
    %======================================================================
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % -------------------      ODE45 in MATLAB         --------------------
    % In this section, gradient is computed using the quasi-analytical formulation as follows:
    
%     tic
    plot_flag	= 1;
    dJdu = gradient(U);
    %plot_flag	= 0;
    norm_gradient	= norm(dJdu, 2);
    fprintf('Iteration No. %3i\tGradient Norm = %1.4e\n', counter, norm_gradient)
%     fprintf('Elapsed time = %1.4f sec\n', toc)
%     	figure(300)
%     	hold on
%     	plot(t_u, dJdu);
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % -------------------      Finite Difference         ------------------
    % In this section,gradient is computed using finite difference as follows:
% %     
%     	tic
%     	Search_Dir = zeros(N+1,1);
%     	du = 1;
%     	f0 = cost(0, U, Search_Dir);
%     	for i=1:N+1
%     		Search_Dir(i) = 1;
%     		f1 = cost(du, U, Search_Dir);
%     		dJdu(i) = (f1-f0)/du;
%     		Search_Dir(i) = 0;
%     	end
%     	plot(t_u, dJdu, 'r.');
%     	fprintf('Elapsed time = %1.4f sec\n', toc)
%     
    %======================================================================
    % Search Direction
    %======================================================================
    % In this section, search direction is  found using gradient as follows: 
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % --------------      Steepest Decsent(Cauchy)         ----------------
    Search_Dir = - dJdu;
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % --------------------      Newton's Method         -------------------
    % This algorithm must be written by the student.
    
    
    
    
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % ----------------      Quasi-Newton (Rank1-Update)        ------------
    % This algorithm must be written by the student.
    
     
     
     
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % ----------------      Quasi-Newton(Rank2-Update)         ------------
    % This algorithm must be written by the student.
    
    
    
    
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % ------------------------     DFP          ---------------------------
    % This algorithm must be written by the student.
    
    
    
    
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % ------------------------      BFGS          -------------------------
    % This algorithm must be written by the student.
    
    
    
  
    %======================================================================
    % Line Search
    %======================================================================
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % -------------------      fixed step length         ------------------
    lambda	= 1e-2/dt;
    
    % test the lambda function
        f = zeros(1);
    	lambda_arr=0:1e-3/dt:4e-1/dt;
    	for i=1:length(lambda_arr)
    		f(i) = cost(lambda_arr(i), U, Search_Dir);
    	end
    	figure
    	plot(lambda_arr, f)
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % ---------------      variable step length         -------------------
    % This algorithm must be written by the student.
    
    
    
  
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % ---------------     Quadratic Interpolation         -----------------
    % This algorithm must be written by the student.
    % lambda tilda is the optimum lambda
%     lambda_tilda = zeros(1);
%     function lambda_tilda = quadratic_interpolation(A, B, C , epsilon)
%         lambda_tilda = 1;
%     end
  
    
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % -------------------      Golden Section         ---------------------
    % This algorithm must be written by the student.
    
    
    
  
    
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % -------------------      Interval Halving         -------------------
    % This algorithm must be written by the student.
    
    
    
  
    
    %======================================================================
    %next step
    %======================================================================
    U	= U + lambda * Search_Dir;
end

end


%==========================================================================
% FUNCTIONS
%==========================================================================
% This function computes model differential equations, that is expressed as:
% xdot=Ax+Bu
% Note: The initial condition is initial state.
function d = diff_equ(t, X)
global A B U_arr t_u
u = interp1(t_u, U_arr, t, 'pchip');
d = A*X + B*u;
end

%==========================================================================
% This function computs cost differential equations.The quadratic 
% continuous-time cost functional is expressed as follows:
% J(u, x(t), t) = 0.5 * x(tf) H x(tf)+ 0.5 * int(x(t)Q(t)x(t) + u(t)R(t)u(t))dt 
% The integral term can be converted to a differential equation as follows:
% xdot3 =  0.5 * x(t)Q(t)x(t) + 0.5* u(t)R(t)u(t)
function d = diff_equ_x_J(t, XX)
global A B U_arr t_u Q R
X = XX(1:2);
u = interp1(t_u, U_arr, t, 'pchip');
d = A*X + B*u;
d(3) = 0.5*X'*Q*X + 0.5*R*u^2;
end

%==========================================================================
% This function computs costate differential equations, that is expressed as:
% pdot=-(d(Hamiltonian)/d(x))
% For the quadratic continuous-time cost functional, pdot is computed
% as :pdot =-A*p-Q*x
% Note: Since x(tf) is free, (dh(tf)/dx(tf))-p(tf)=0. Therefore, boundry  
% condition of costate differential equations is as follows:
% p(tf)= dh(tf)/dx(tf).
function d = diff_equ_p(t, p)
global A x_global Q time_x
x = interp1(time_x, x_global, t, 'pchip');
d = -A'*p - Q*x';
end

%==========================================================================
% This function computs gradient of the cost.
% dJ=(d(Hamiltonian)/du)*dt; 
% For the quadratic continuous-time cost functional,gradient of cost is 
% expressed as follows: 
% d(Hamiltonian)/du = R*u + p*B and dJ=(R*u + p*B)*dt
function dj = gradient(u)
global x0 H tf dt B R plot_flag
options = odeset('AbsTol', 1e-6, 'RelTol', 1e-6);
global U_arr t_u time_x x_global
U_arr = u;
% tic
% [time_x, x_global] = ode45(@diff_equ, [0 tf], x0, options);
% toc
% tic
[time_x, x_global] = ode45(@diff_equ, t_u, x0, options);
% toc
if plot_flag==1
    figure(200)
    hold on
    plot(time_x, x_global)
	drawnow()
end
n = length(time_x);
p_tf = H*x_global(n,:)';
% [time_p, p] = ode45(@diff_equ_p, [tf:-dt:0], p_tf, options);
[time_p, p] = ode45(@diff_equ_p, [tf 0], p_tf, options);
p = interp1(time_p, p, t_u, 'pchip');
% figure(100)
% plot(time_p, p,'b*')
% plot(t_u, p,'r.')
% hold on
% n = length(time_p);
% time_p = time_p(n:-1:1);
% p = p(n:-1:1,:);
Hu = R*u + p*B;
dj = Hu*dt;
end

%==========================================================================
% This function computs the cost. In optimal control theory, the cost 
% function is expressed as follows:
% J(u, x(t), t) =h(x(tf),tf)+int(g(x(t),u(t),t)dt
% The quadratic continuous-time cost functional is expressed as follows:
% J(u, x(t), t) = 0.5 * x(tf) H x(tf)+ 0.5 * int(x(t)Q(t)x(t) + u(t)R(t)u(t))dt
% The integral term can be converted to a differential equation as follows:
% xdot3 =  0.5 * x(t)Q(t)x(t) + 0.5* u(t)R(t)u(t)
% Therfore, the quadratic  cost functional can be expressed as follows:
% J(u, x(t), t) = 0.5 * x(tf) H x(tf)+ xdot3
function J = cost(lambda, u, S)
global x0 H tf
global U_arr
options = odeset('AbsTol', [1e-6 1e-6 1e-6], 'RelTol', 1e-8);

U_arr = u + lambda*S;
%xx0 = [x0;0]; % zero is for cost function integral
[~, x] = ode45(@diff_equ_x_J, [0 tf], [x0;0], options);
xend = x(end, 1:2)';
J = x(end, 3) + 0.5*xend'*H*xend;
end
%==========================================================================

