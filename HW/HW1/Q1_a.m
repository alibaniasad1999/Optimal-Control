%% Plot figure %%
[X,Y] = meshgrid(linspace(-4, 4));
Z = Y .* sin(X + Y) - X .* sin(X - Y);
figure('Name','Mesh Grid','NumberTitle','off');
mesh(X, Y, Z)
figure('Name','Contour','NumberTitle','off');
contour(X, Y, Z)
%% solve f Gradaian %%
% x and y decelration %
syms x y
% write equations %
eq1 = y * cos(x - y) - sin(x - y) - x * cos(x - y);
eq2 = y * cos(x + y) + sin(x + y) + x * cos(x - y);
eq = [eq1, eq2];
ans_counter = 1;
sol_x = zeros(1);
sol_y = zeros(1);
for i = -3:3
    for j = -3:3
        [ans_x, ans_y] = vpasolve(eq, [x,y], [i; j]);
        if ans_counter > 1 && abs(sol_x(ans_counter-1) - ans_x) < 1e-1 && abs(sol_y(ans_counter-1) - ans_y) < 1e-1
            continue;
        end
        if ans_x > 4 || ans_x < -4 || ans_y > 4 || ans_y < -4
            continue;
        end
        sol_x(ans_counter) = ans_x;
        sol_y(ans_counter) = ans_y;
        ans_y * cos(ans_x - ans_y) - sin(ans_x - ans_y) - ans_x * cos(ans_x - ans_y);
        ans_y * cos(ans_x + ans_y) + sin(ans_x + ans_y) + ans_x * cos(ans_x - ans_y);
        ans_counter = ans_counter + 1;
    end
end
sol = [sol_x; sol_y];

