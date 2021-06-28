X_zero = [-1; 1];
X      = X_zero; % current  X
X_prev = X_zero; % previous X
gradient_tol = 1e-8; 
li_tol = 1e-3; % linear serach
dJ = gradient(X)';
dJ_prev = gradient(X_prev)';
epsilon = 0.01;
% BFGS + Golden Section
%{
while abs(dJ) > gradient_tol
    if isequal(X, X_prev)
        B = eye(2);
        search_dir = -B * dJ;
        [a, b, c, ~, f_b, ~] = bracketing(X, search_dir, epsilon);
        lambda = golden_section(a, b, c, f_b, search_dir, X, li_tol);
        X_prev = X;
        X = X + lambda * search_dir;
        dJ_prev = dJ;
        dJ = gradient(X)';
        continue
    end
    d =  X -  X_prev;
    g = dJ - dJ_prev;
    search_dir = BFGS(B, d, g, dJ);
    [a, b, c, ~, f_b, ~] = bracketing(X, search_dir, epsilon);
    lambda = golden_section(a, b, c, f_b, search_dir, X, lr_tol);
    X_prev = X;
    X = X + lambda * search_dir;
    dJ_prev = dJ;
    dJ = gradient(X)';
end
%}
% BFGS + Quadratic Interpolation
%{
while abs(dJ) > gradient_tol
    if isequal(X, X_prev)
        B = eye(2);
        search_dir = -B * dJ;
        [a, b, c, ~, f_b, ~] = bracketing(X, search_dir, epsilon);
        lambda = quadratic_interpolation(a, b, c, search_dir, X, li_tol);
        X_prev = X;
        X = X + lambda * search_dir;
        dJ_prev = dJ;
        dJ = gradient(X)';
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
end
%}
% Steepest Descent + Quadratic Interpolation

while abs(dJ) > gradient_tol
    search_dir = steepest_decsent(dJ);
    [a, b, c, ~, f_b, ~] = bracketing(X, search_dir, epsilon);
    if ~(a == 0 && b == 0 && c == 0)
        lambda = quadratic_interpolation(a, b, c ,search_dir, X, li_tol);
    end
    X = X + lambda * search_dir;
    dJ = gradient(X)';
end

% Steepest Descent + Golden Section
%{
while abs(dJ) > gradient_tol
    search_dir = steepest_decsent(dJ);
    [a, b, c, ~, f_b, ~] = bracketing(X, search_dir, epsilon);
    if ~(a == 0 && b == 0 && c == 0)
        lambda = golden_section(a, b, c, f_b, search_dir, X, li_tol);
    end
    X = X + lambda * search_dir;
    dJ = gradient(X)';
end
%}
%==========================================================================
% FUNCTIONS
%==========================================================================
% Cost function %
function J = cost(X)
    x = X(1);
    y = X(2);
    J = y * sin(x + y) - x * sin(x - y);
end
function dJ = gradient(X)
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
    if f_a < f_b
        search_dir = -search_dir;
        X_a = X + a * search_dir;
        X_b = X + b * search_dir;
        f_a = cost(X_a);
        f_b = cost(X_b);
        if abs(f_a - f_b) > 0.01
            disp('change epsilon number');
            return;
        else
            a   = 0;
            b   = 0;
            c   = 0;
            f_a = 0;
            f_b = 0;
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
function lambda_tilda = quadratic_interpolation(A, B, C, search_dir,...
    X, tol)
	X_a = X + A * search_dir;
    X_b = X + B * search_dir;
    X_c = X + C * search_dir;
    f_a = cost(X_a);
    f_b = cost(X_b);
    f_c = cost(X_c);
    % three equation and three unknown %
    a = (f_a * B * C * (C - B) + f_b * C * A * (A - C) + f_c * A * B * (B - A))...
        /((A - B) * (B - C) * (C - A));
    b = (f_a * (B^2 - C^2) + f_b * (C^2 - A^2) + f_c * (A^2 - B^2))...
        /((A - B) * (B - C) * (C - A));
    c = -(f_a * (B - C) + f_b * (C - A) + f_c * (A - B))...
        /((A - B) * (B - C) * (C - A));
    lambda_tilda = -b / (2 * c);
    % Quadratic function h(lambda) = a + b * lambda + c * lambda^2
    h = a + b * lambda_tilda + c * lambda_tilda^2;
    X_lambda = X + lambda_tilda * search_dir;
    f_lambda_tilda = cost(X_lambda);
    while abs((h - f_lambda_tilda) / f_lambda_tilda) >= tol
        if lambda_tilda > B && f_lambda_tilda < f_b
            A = B;
            B = lambda_tilda;
        elseif lambda_tilda > B && f_lambda_tilda > f_b
            C = lambda_tilda;
        elseif lambda_tilda < B && f_lambda_tilda < f_b
            C = B;
            B = lambda_tilda;
        else
            A = lambda_tilda;
        end
        X_a = X + A * search_dir;
        X_b = X + B * search_dir;
        X_c = X + C * search_dir;
        f_a = cost(X_a);
        f_b = cost(X_b);
        f_c = cost(X_c);
        % three equation and three unknown %
        a = (f_a * B * C * (C - B) + f_b * C * A * (A - C) + f_c * A * B * (B - A))...
            /((A - B) * (B - C) * (C - A));
        b = (f_a * (B^2 - C^2) + f_b * (C^2 - A^2) + f_c * (A^2 - B^2))...
            /((A - B) * (B - C) * (C - A));
        c = -(f_a * (B - C) + f_b * (C - A) + f_c * (A - B))...
            /((A - B) * (B - C) * (C - A));
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