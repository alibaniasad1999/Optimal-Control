X1_zero = [-1; -1];
X1      = X1_zero; % current  X
X1_prev = X1_zero; % previous X
%==========================================================================
% FUNCTIONS
%==========================================================================
% Cost function %
function J = cost(X)
    x = X(1);
    y = X(2);
    J = y * sin(x + y) - x * sin(x - y);
end
function dJ = gradient(x, y)
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
    search_dir = B * dJ;
end
%======================================================================
% Bracketing
%======================================================================
% In this section, Bracket is  found using golden number(1.618)
function [a, b, c] = bracketing(X, search_dir, epsilon)
    a =       0;
    b = epsilon;
    X_a = X + a * search_dir;
    X_b = X + b * search_dir;
    if cost(X_a) < X_a
        disp('change epsilon or search direction')
        return;
    end
    gamma = 1.618; % golden number
    c = b + gamma * (b - a);
    X_c = X + c * search_dir;
    while cost(X_b) > cost(X_c)
        a = b;
        b = c;
        c = b + gamma * (b - a);
    end
end
%----------------------------------------------------------------------
%----------------------------------------------------------------------
%----------------------------------------------------------------------
% ---------------     Quadratic Interpolation         -----------------
function lambda_tilda = quadratic_interpolation(A, B, C, search_dir,...
    X, epsilon)
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
    while abs((h - f_lambda_tilda) / f_lambda_tilda) >= epsilon
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