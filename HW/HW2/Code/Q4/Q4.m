%%%% ODE Question 4 %%%%
global A B
A = [-1  0 ;
      0 -1];
B = [ 1  1]';
[t, x] = ode45(@diff_eq_states,[0, 10], [10 10]');
%% System Functions %%
function d = diff_eq_states(~, x)
global A B
%%% Switching function %%%
s = x(1) + 0.5 * x(2) * abs(x(2));
u = 0;
if     s > 0
    u = -1;
elseif s < 0
    u =  1;
else
    if     x(2) > 0
        u = -1;
    elseif x(2) < 0
        u =  1;
    end
end
d	= A*x + B*u;
end