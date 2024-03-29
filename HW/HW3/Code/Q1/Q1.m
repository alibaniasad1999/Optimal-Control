%%% plot figure %%%
X_1 = linspace(-2, -1, 100);
Y_1 = linspace(-1,   1, 100);
[X_1,Y_1] = meshgrid(X_1, Y_1);
Z_1 = Y_1 .* sin(X_1 + Y_1) - X_1 .* sin(X_1 - Y_1);
figure1 = figure('Name','Contour','NumberTitle','off');
contour(X_1, Y_1, Z_1, 200)
xlabel('x')
ylabel('y')
hold on;
X_zero = [-1; -1];
X      = X_zero; % current  X
X_prev = X_zero; % previous X
gradient_tol = 1e-7;
li_tol = 1e-3; % linear serach
global cost_iteration gradient_iteration
cost_iteration = 0;
gradient_iteration = 0;
dJ = gradient(X)';
dJ_prev = gradient(X_prev)';
epsilon = 0.01;
counter = 0;
counter = counter + 1;
choice = menu('Choose Method','Steepest Descent + Quadratic Interpolation'...
    ,'Steepest Descent + Golden Section', 'BFGS + Quadratic Interpolation'...
    , 'BFGS + Golden Section');
if choice == 4
    % BFGS + Golden Section
    tic
    while norm(dJ, 2) > gradient_tol
        counter = counter + 1;
        fprintf('Iteration No. %3i\tGradient Norm = %1.4e\n', counter, norm(dJ, 2))
        if isequal(X, X_prev)
            B = eye(2);
            search_dir = -B * dJ;
            [a, b, c, ~, f_b, ~] = bracketing(X, search_dir, epsilon);
            lambda = golden_section(a, b, c, f_b, search_dir, X, li_tol);
            X_prev = X;
            X = X + lambda * search_dir;
            dJ_prev = dJ;
            dJ = gradient(X)';
            x(1) = X_prev(1);
            y(1) = X_prev(2);
            x(2) = X(1);
            y(2) = X(2);
            quiver(x(1), y(1), x(2)-x(1), y(2)-y(1), 1)
            continue
        end
        d =  X -  X_prev;
        g = dJ - dJ_prev;
        search_dir = BFGS(B, d, g, dJ);
        [a, b, c, ~, f_b, ~] = bracketing(X, search_dir, epsilon);
        lambda = golden_section(a, b, c, f_b, search_dir, X, li_tol);
        X_prev = X;
        X = X + lambda * search_dir;
        dJ_prev = dJ;
        dJ = gradient(X)';
        x(1) = X_prev(1);
        y(1) = X_prev(2);
        x(2) = X(1);
        y(2) = X(2);
        quiver(x(1), y(1), x(2)-x(1), y(2)-y(1), 1)
    end
    fprintf('Elapsed time = %1.8f sec\n', toc)
end
if choice == 3
    tic
    % BFGS + Quadratic Interpolation
    while norm(dJ, 2) > gradient_tol
        counter = counter + 1;
        fprintf('Iteration No. %3i\tGradient Norm = %1.4e\n', counter, norm(dJ, 2))
        if isequal(X, X_prev)
            B = eye(2);
            search_dir = -B * dJ;
            [a, b, c, ~, f_b, ~] = bracketing(X, search_dir, epsilon);
            lambda = quadratic_interpolation(a, b, c, search_dir, X, li_tol);
            X_prev = X;
            X = X + lambda * search_dir;
            dJ_prev = dJ;
            dJ = gradient(X)';
            x(1) = X_prev(1);
            y(1) = X_prev(2);
            x(2) = X(1);
            y(2) = X(2);
            quiver(x(1), y(1), x(2)-x(1), y(2)-y(1), 1)
            continue
        end
        d =  X -  X_prev;
        g = dJ - dJ_prev;
        search_dir = BFGS(B, d, g, dJ);
        [a, b, c, ~, f_b, ~] = bracketing(X, search_dir, epsilon);
        if ~(a == 0 && b == 0 && c == 0)
            lambda = golden_section(a, b, c, f_b, search_dir, X, li_tol);
        end
        X_prev = X;
        X = X + lambda * search_dir;
        dJ_prev = dJ;
        dJ = gradient(X)';
        x(1) = X_prev(1);
        y(1) = X_prev(2);
        x(2) = X(1);
        y(2) = X(2);
        quiver(x(1), y(1), x(2)-x(1), y(2)-y(1), 1)
    end
    fprintf('Elapsed time = %1.8f sec\n', toc)
end

if choice == 1
    tic
    % Steepest Descent + Quadratic Interpolation
    while norm(dJ, 2) > gradient_tol
        counter = counter + 1;
        fprintf('Iteration No. %3i\tGradient Norm = %1.4e\n', counter, norm(dJ, 2))
        X_prev = X;
        search_dir = steepest_decsent(dJ);
        [a, b, c, ~, f_b, ~] = bracketing(X, search_dir, epsilon);
        if ~(a == 0 && b == 0 && c == 0)
            lambda = quadratic_interpolation(a, b, c ,search_dir, X, li_tol);
        end
        X = X + lambda * search_dir;
        dJ = gradient(X)';
        x(1) = X_prev(1);
        y(1) = X_prev(2);
        x(2) = X(1);
        y(2) = X(2);
        quiver(x(1), y(1), x(2)-x(1), y(2)-y(1), 1)
    end
    fprintf('Elapsed time = %1.8f sec\n', toc)
end

if choice == 2
    tic
    % Steepest Descent + Golden Section
    while abs(dJ) > gradient_tol
        X_prev = X;
        counter = counter + 1;
        fprintf('Iteration No. %3i\tGradient Norm = %1.4e\n', counter, norm(dJ, 2))
        search_dir = steepest_decsent(dJ);
        [a, b, c, ~, f_b, ~] = bracketing(X, search_dir, epsilon);
        if ~(a == 0 && b == 0 && c == 0)
            lambda = golden_section(a, b, c, f_b, search_dir, X, li_tol);
        end
        X = X + lambda * search_dir;
        dJ = gradient(X)';
        x(1) = X_prev(1);
        y(1) = X_prev(2);
        x(2) = X(1);
        y(2) = X(2);
        quiver(x(1), y(1), x(2)-x(1), y(2)-y(1), 1)
    end
    fprintf('Elapsed time = %1.8f sec\n', toc)
end
%%% printer %%%
switch choice
    case 1
        fprintf('Steepest Descent + Quadratic Interpolation--> gradient iteration = %.0f, cost_iteration = %.0f\n'...
            , gradient_iteration, cost_iteration);
        print(figure1, '../../Figure/Q1/part a Steepest Descent + Quadratic Interpolation.png','-dpng','-r300')
    case 2
        fprintf('Steepest Descent + Golden Section--> gradient iteration = %.0f, cost_iteration = %.0f\n'...
            , gradient_iteration, cost_iteration);
        print(figure1, '../../Figure/Q1/part a Steepest Descent + Golden Section.png','-dpng','-r300')
    case 3
        fprintf('BFGS + Quadratic Interpolation--> gradient iteration = %.0f, cost_iteration = %.0f\n'...
            , gradient_iteration, cost_iteration);
        print(figure1, '../../Figure/Q1/part a BFGS + Quadratic Interpolation.png','-dpng','-r300')
    otherwise
        fprintf('BFGS + Golden Section--> gradient iteration = %.0f, cost_iteration = %.0f\n'...
            , gradient_iteration, cost_iteration);
        print(figure1, '../../Figure/Q1/part a BFGS + Golden Section.png','-dpng','-r300')
end
%==========================================================================
% FUNCTIONS
%==========================================================================
% Cost function %
function J = cost(X)
global cost_iteration
cost_iteration = cost_iteration + 1;
x = X(1);
y = X(2);
J = y * sin(x + y) - x * sin(x - y);
end
function dJ = gradient(X)
global gradient_iteration
gradient_iteration = gradient_iteration + 1;
x = X(1);
y = X(2);
% df/dx
dJ(1) = y * cos(x + y) - sin(x - y) - x * cos(x - y);
% df/dy
dJ(2) = y * cos(x + y) + sin(x + y) + x * cos(x - y);
end
%======================================================================
% Search Direction
%======================================================================
% In this section, search direction is  found using gradient as follows:
%----------------------------------------------------------------------
%----------------------------------------------------------------------
%----------------------------------------------------------------------
% --------------      Steepest Decsent(Cauchy)         ----------------
function search_dir = steepest_decsent(dJ)
search_dir = - dJ;
end
% -------------------------      BFGS         --------------------------
function search_dir = BFGS(B, d, g, dJ)
% d =  X -  X_prev
% g = dJ - dJ_prev
B = B + d * d' / (d' * g) * (1 + g' * B * g / (d' * g)) - ...
    B * g * d' / (d' * g) - d * g' * B /  (d' * g);
search_dir = -B * dJ;
end
%======================================================================
% Bracketing
%======================================================================
% In this section, Bracket is  found using golden number(1.618)
function [a, b, c, f_a, f_b, f_c] = bracketing(X, search_dir, epsilon)
a =       0;
b = epsilon;
X_a = X + a * search_dir;
X_b = X + b * search_dir;
f_a = cost(X_a);
f_b = cost(X_b);
counter = 1;
while f_a <= f_b
    counter = counter + 1;
    epsilon = epsilon / 2;
    b = epsilon;
    X_b = X + b * -search_dir;
    f_b = cost(X_b);
    if counter == 100
        break
    end
    if b == 0
        c = 0;
        f_c = 0;
        return;
    end
    
end
gamma = 1.618; % golden number
c = b + gamma * (b - a);
X_c = X + c * search_dir;
f_c = cost(X_c);
while f_b > f_c
    a = b;
    b = c;
    c = b + gamma * (b - a);
    X_a = X_b;
    X_b = X_c;
    X_c = X + c * search_dir;
    f_b = cost(X_b);
    f_c = cost(X_c);
end
f_a = cost(X_a);
end
%----------------------------------------------------------------------
%----------------------------------------------------------------------
%----------------------------------------------------------------------
% ---------------     Quadratic Interpolation         -----------------
function lambda_tilda = quadratic_interpolation(A_QI, B_QI, C_QI, search_dir,...
    X, tol)
X_a = X + A_QI * search_dir;
X_b = X + B_QI * search_dir;
X_c = X + C_QI * search_dir;
f_a = cost(X_a);
f_b = cost(X_b);
f_c = cost(X_c);
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
X_lambda = X + lambda_tilda * search_dir;
f_lambda_tilda = cost(X_lambda);
while abs((h - f_lambda_tilda) / f_lambda_tilda) >= tol
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
    X_a = X + A_QI * search_dir;
    X_b = X + B_QI * search_dir;
    X_c = X + C_QI * search_dir;
    f_a = cost(X_a);
    f_b = cost(X_b);
    f_c = cost(X_c);
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
    X_lambda = X + lambda_tilda * search_dir;
    f_lambda_tilda = cost(X_lambda);
end
end
%----------------------------------------------------------------------
%----------------------------------------------------------------------
%----------------------------------------------------------------------
% -------------------      Golden Section         ---------------------
function lambda_tilda = golden_section(a, b, c, f_b,...
    search_dir, X, tol)
% cuase we use golden number in bracketing a = a b = x_1 c = b
% a is lower and b is upper
% initial condition from bracketing
goldnum = .618;
f_1 = f_b;
%f_b = f_c;
x_1 = b;
b   = c;
d   = (b - a) * goldnum;
x_2 = a + d;
X_2 = X + x_2 * search_dir;
f_2 = cost(X_2);
while abs(a - b) > tol
    if f_1 > f_2
        % new a
        a   = x_1;
        %f_a = f_1;
        % new x_1
        x_1 = x_2;
        f_1 = f_2;
        % new x_2
        d   = (b - a) * goldnum;
        x_2 = a + d;
        X_2 = X + x_2 * search_dir;
        f_2 = cost(X_2);
    elseif f_2 > f_1
        % new b
        b = x_2;
        %f_b = f_2;
        % new x_2
        x_2 = x_1;
        f_2 = f_1;
        % new x_1
        d   = (b - a) * goldnum;
        x_1 = b - d;
        X_1 = X + x_1 * search_dir;
        f_1 = cost(X_1);
    else
        a = (x_1 + x_2) / 2;
        b = a;
    end
end
lambda_tilda = (x_1 + x_2) / 2;
end