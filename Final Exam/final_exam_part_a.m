%-----------------------------%%%% Q2_a %%%%-------------------------------
%--------------------------------------------------------------------------
% 1392/2/10 (April, 30th, 2013)
% by Nima Assadian and Alireza Sharifi and Ali Baniasad
% Department of Aerospace Engineering
% Sharif University of Technology
%--------------------------------------------------------------------------
% This program is designed for optimal control course. This course is
% intended for students of engineering.
%--------------------------------------------------------------------------
%==========================================================================
% Main Routine
%==========================================================================
function OptimalControl_Optimization_b()
clc
warning off;
global tf t_u R Q H A B x0 dt plot_flag counter r_constrain
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

A	= [0	1
      -1 -0.1];
B	= [0
       1];
x0  = [1 ;
       1];
epsilon = 0.01; % Bracketing
% -------------------      Cost Function         --------------------------
%  A control problem includes a cost functional that is a function of state
%  and control variables.In optimal control theory, the following objective
%  function is minimized:
% J(u, x(t), t) =h(x(tf),tf)+int(g(x(t),u(t),t)dt
% A special case of the general nonlinear optimal control problem given in
% the previous section is the linear quadratic regulator(LQR) optimal
% control problem. The LQR problem is stated as follows.
% Minimize the quadratic continuous-time cost functional
% J(u, x(t), t) = 0.5 * x(tf) H x(tf)+ 0.5 * int(x(t)Q(t)x(t) +
% u(t)R(t)u(t))dt + g(u) (constrain) + g(x)(constrain)
% where Q(t) and H are symmetric positive semi-definite n * n matrices,
% R(t) is a symmetric positive definite m*m matrix.
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
H		= 10 * eye(2);
Q		= 1  * eye(2);
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
U_prev  = U; 
t_u		= 0:dt:tf;
% -------------------      Execution Options         ----------------------
plot_flag	= 1;

%==========================================================================
%  Optimization Loop
%==========================================================================
tol				= 1e-4;
tol_lambda      = 1e-2;
norm_gradient	= tol + 1;
max_count		= 256;
counter			= 0;
U_saver = zeros(500, N+1);
choice = menu('Choose Method','Steepest Descent + Quadratic Interpolation'...
    ,'Steepest Descent + Golden Section', 'BFGS + Quadratic Interpolation'...
    , 'BFGS + Golden Section');
while (norm_gradient > tol && counter < max_count)
    counter = counter + 1;
    if counter == 1
        r_constrain = .1;
    else
        if r_constrain > 1e-6
            r_constrain = r_constrain * 0.5;
        end
    end
    %======================================================================
    % Gradient
    %======================================================================
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % -------------------      ODE45 in MATLAB         --------------------
    % In this section, gradient is computed using the quasi-analytical formulation as follows:
    
    tic
    plot_flag	= 1;
    if counter == 1
        dJdu      = gradient(U);
        dJdu_prev = dJdu;
    else
        dJdu_prev = dJdu;
        dJdu = gradient(U);
    end
    U_saver(counter, :) = U;
%     plot_flag	= 0;
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
%     	du = 0.01;
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
    if choice == 1 || choice == 2
        Search_Dir = - dJdu;
    end
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
    if choice == 3 || choice == 4
        d =  U   - U_prev   ;
        g = dJdu - dJdu_prev;
        if counter == 1
            B_BFGS = eye(N+1);
        else
            B_BFGS = B_BFGS + d * d' / (d' * g) * (1 + g' * B_BFGS * g / (d' * g)) - ...
                B_BFGS * g * d' / (d' * g) - d * g' * B_BFGS /  (d' * g);
        end
%     figure(200)
%     hold on
%     plot(time_x, x_global)
% 	drawnow()
    Search_Dir = -B_BFGS * dJdu;
    end
    %======================================================================
    % Line Search
    % Bracketing
    %======================================================================
    % In this section, Bracket is  found using golden number(1.618)
    [lower, middle, upper, cost_lower, cost_middle, cost_upper] = ...
        bracketing(U, Search_Dir, epsilon);
    %======================================================================
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % -------------------      fixed step length         ------------------
%     lambda	= 1e-2/dt;
    
    % test the lambda function
%     	lambda_arr=0:1e-3/dt:4e-1/dt;
%         f = zeros(1);
%     	for i=1:length(lambda_arr)
%     		f(i) = cost(lambda_arr(i), U, Search_Dir);
%     	end
%     	figure
%     	plot(lambda_arr, f)
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % ---------------      variable step length         -------------------
    % This algorithm must be written by the student.
    
    
    
  
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % ---------------     Quadratic Interpolation         -----------------
    if choice == 1 || choice == 3
        A_QI = lower;
        B_QI = middle;
        C_QI = upper;
        %     U_a = U + A_QI * search_dir;
        %     U_b = U + B_QI * search_dir;
        %     U_c = U + C_QI * search_dir;
        f_a = cost_lower;
        f_b = cost_middle;
        f_c = cost_upper;
        % three equation and three unknown %
        a = (f_a * B_QI * C_QI * (C_QI - B_QI) + f_b * C_QI * A_QI * (A_QI - C_QI) + f_c * A_QI * B_QI * (B_QI - A_QI))...
            /((A_QI - B_QI) * (B_QI - C_QI) * (C_QI - A_QI));
        b = (f_a * (B_QI^2 - C_QI^2) + f_b * (C_QI^2 - A_QI^2) + f_c * (A_QI^2 - B_QI^2))...
            /((A_QI - B_QI) * (B_QI - C_QI) * (C_QI - A_QI));
        c = -(f_a * (B_QI - C_QI) + f_b * (C_QI - A_QI) + f_c * (A_QI - B_QI))...
            /((A_QI - B_QI) * (B_QI - C_QI) * (C_QI - A_QI));
        lambda_tilda = -b / (2 * c);
        % Quadratic function h(lambda) = a + b * lambda + c * lambda^2
        h = a + b * lambda_tilda + c * lambda_tilda^2;
        %     U_lambda = U + lambda_tilda * Search_Dir;
        f_lambda_tilda = cost(lambda_tilda, U, Search_Dir);
        while abs((h - f_lambda_tilda) / f_lambda_tilda) >= tol_lambda
            if lambda_tilda > B_QI && f_lambda_tilda < f_b
                A_QI = B_QI;
                B_QI = lambda_tilda;
            elseif lambda_tilda > B_QI && f_lambda_tilda > f_b
                C_QI = lambda_tilda;
            elseif lambda_tilda < B_QI && f_lambda_tilda < f_b
                C_QI = B_QI;
                B_QI = lambda_tilda;
            else
                A_QI = lambda_tilda;
            end
            %         U_a = U + A_QI * Search_Dir;
            %         U_b = U + B_QI * Search_Dir;
            %         U_c = U + C_QI * Search_Dir;
            f_a = cost(A_QI, U, Search_Dir);
            f_b = cost(B_QI, U, Search_Dir);
            f_c = cost(C_QI, U, Search_Dir);
            % three equation and three unknown %
            a = (f_a * B_QI * C_QI * (C_QI - B_QI) + f_b * C_QI * A_QI * (A_QI - C_QI) + f_c * A_QI * B_QI * (B_QI - A_QI))...
                /((A_QI - B_QI) * (B_QI - C_QI) * (C_QI - A_QI));
            b = (f_a * (B_QI^2 - C_QI^2) + f_b * (C_QI^2 - A_QI^2) + f_c * (A_QI^2 - B_QI^2))...
                /((A_QI - B_QI) * (B_QI - C_QI) * (C_QI - A_QI));
            c = -(f_a * (B_QI - C_QI) + f_b * (C_QI - A_QI) + f_c * (A_QI - B_QI))...
                /((A_QI - B_QI) * (B_QI - C_QI) * (C_QI - A_QI));
            lambda_tilda = -b / (2 * c);
            % Quadratic function h(lambda) = a + b * lambda + c * lambda^2
            h = a + b * lambda_tilda + c * lambda_tilda^2;
            %         U_lambda = U + lambda_tilda * Search_Dir;
            f_lambda_tilda = cost(lambda_tilda, U, Search_Dir);
        end
        lambda = lambda_tilda;
    end
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % -------------------      Golden Section         ---------------------
    % cuase we use golden number in bracketing a = a b = x_1 c = b
    % a is lower and b is upper
    % initial condition from bracketing
    if choice == 2 || choice == 4
        
        goldnum = .618;
        a = lower;
        f_1 = cost_middle;
        %f_b = f_c;
        u_1 = middle;
        b   = upper;
        d   = (b - a) * goldnum;
        u_2 = a + d;
        %U_2 = U + u_2 * Search_Dir;
        f_2 = cost(u_2, U, Search_Dir);
        while abs(a - b) > tol_lambda
            if f_1 > f_2
                % new a
                a   = u_1;
                %f_a = f_1;
                % new x_1
                u_1 = u_2;
                f_1 = f_2;
                % new x_2
                d   = (b - a) * goldnum;
                u_2 = a + d;
                %U_2 = U + u_2 * Search_Dir;
                f_2 = cost(u_2, U, Search_Dir);
            elseif f_2 > f_1
                % new b
                b = u_2;
                %f_b = f_2;
                % new x_2
                u_2 = u_1;
                f_2 = f_1;
                % new x_1
                d   = (b - a) * goldnum;
                u_1 = b - d;
                %U_1 = U + u_1 * Search_Dir;
                f_1 = cost(u_1, U, Search_Dir);
            else
                a = (u_1 + u_2) / 2;
                b = a;
            end
        end
        lambda = (u_1 + u_2) / 2;
    end
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    %----------------------------------------------------------------------
    % -------------------      Interval Halving         -------------------
    % This algorithm must be written by the student.
    
    
    
  
    
    %======================================================================
    %next step
    %======================================================================
    
    norm_gradient	= norm(dJdu, 2);
    fprintf('Iteration No. %3i\tGradient Norm = %1.4e\n', counter, norm_gradient)
    fprintf('Elapsed time = %1.4f sec\n', toc)
    fprintf('Iteration No. %3i\tSearch direction * lambda = %1.4e\n', counter, norm(dJdu - dJdu_prev, 2))
    U_prev = U;
    U	   = U + lambda * Search_Dir;
end
save U_3.mat U_saver;
switch choice
    case 1
        print(200, 'Constrain Steepest Descent + Quadratic Interpolation.png','-dpng','-r300')
    case 2
        print(200, 'Constrain Steepest Descent + Golden Section.png','-dpng','-r300')
    case 3
        print(200, 'Constrain BFGS + Quadratic Interpolation.png','-dpng','-r300')
    otherwise
        print(200, 'Constrain BFGS + Golden Section.png','-dpng','-r300')
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
% xdot3 =  0.5 * x(t)Q(t)x(t) + 0.5* u(t)R(t)u(t) + G(constrain)
function d = diff_equ_x_J(t, XX)
global A B U_arr t_u Q R
X = XX(1:2);
u = interp1(t_u, U_arr, t, 'pchip');
d = A*X + B*u;
% G_cost = zeros(1);
% for j = 1:length(u)
%     [G_cost(j), ~] = G(u(j));
% end
[G1_cost, ~] = G1(u);
[G2_cost, ~] = G2(u);
d(3) = 0.5*X'*Q*X + 0.5*R*u^2 + G1_cost + G2_cost;
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
    xlabel('$Time_{\sec}$', 'interpreter', 'latex');
    ylabel('$\vec{X}$', 'interpreter', 'latex');
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
dg1 = zeros(1);
for i = 1:length(u)
    [~, dg1(i)] = G1(u(i));
end
dg2 = zeros(1);
for i = 1:length(u)
    [~, dg2(i)] = G2(u(i));
end
Hu = R*u + p*B + dg1' + dg2'; % exterior and interior
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
%======================================================================
% Bracketing
%======================================================================
% In this section, Bracket is  found using golden number(1.618)
function [a, b, c, f_a, f_b, f_c] = bracketing(X, search_dir, epsilon)
%global counter;
    a =       0;
    b = epsilon;
    %X_a = X + a * search_dir;
    %X_b = X + b * search_dir;
    f_a = cost(a, X, search_dir);
    f_b = cost(b, X, search_dir);
    if f_a < f_b
%         search_dir = -search_dir;
        %f_a = cost(a, X, search_dir);
%         f_b = cost(b, X, search_dir);
        if f_a < f_b
            search_dir = -search_dir;
            while f_a < f_b
                epsilon = epsilon / 2;
                b = epsilon;
                f_b = cost(b, X, search_dir);
            end
        end
        if b == 0
            disp('change epsilon number');
            return;
        end
    end
    gamma = 1.618; % golden number
    c = b + gamma * (b - a);
    %X_c = X + c * search_dir;
    f_c = cost(c, X, search_dir);
    while f_b >= f_c
        a = b;
        b = c;
        c = b + gamma * (b - a);
        %X_a = X_b;
        %X_b = X_c;
        %X_c = X + c * search_dir;
        f_b = cost(b, X, search_dir);
        f_c = cost(c, X, search_dir);
    end
    f_a = cost(a, X, search_dir);
end
function [cost, dg] = G1(u)
global r_constrain
G = u - 0.4;
% c = 0.9 , a = 1/2
epsilon = -0.9 * (r_constrain) ^ 0.5;
if G <= epsilon 
    cost = -r_constrain / G;
    dg = r_constrain / (u - 0.4)^2;
else
    cost = -r_constrain / epsilon * (3 - 3 * G / epsilon + (G / epsilon)^2);
    dg = -r_constrain / epsilon * (-3 / epsilon + (2 * u - 0.8) / epsilon^2);
end
end
function [cost, dg] = G2(u)
global r_constrain
G = -u - 0.4;
% c = 0.9 , a = 1/2
epsilon = -0.9 * (r_constrain) ^ 0.5;
if G <= epsilon 
    cost = -r_constrain * 1 / G;
    dg = -r_constrain * 1 / (u + 0.4)^2;
else
    cost = -r_constrain  / epsilon * (3 - 3 * G / epsilon + (G / epsilon)^2);
    dg = -r_constrain / epsilon * (3 / epsilon + (2 * u + 0.8) / epsilon^2);
end
end