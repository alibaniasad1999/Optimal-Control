%% Plot figure %%
[X,Y] = meshgrid(linspace(-4, 4));
Z = Y .* sin(X + Y) - X .* sin(X - Y);
figure1 = figure('Name','Mesh Grid','NumberTitle','off');
mesh(X, Y, Z)
xlabel('x')
ylabel('y')
zlabel('z')
figure2 = figure('Name','Contour','NumberTitle','off');
contour(X, Y, Z)
xlabel('x')
ylabel('y')
%% solve f Gradaian %%
% x and y decelration %
syms x y
% write equations %
% df/dx
eq1 = y * cos(x + y) - sin(x - y) - x * cos(x - y);
% df/dy
eq2 = y * cos(x + y) + sin(x + y) + x * cos(x - y);
eq = [eq1, eq2];
ans_counter = 1;
sol_x = zeros(1);
sol_y = zeros(1);
H_ans = zeros(1);
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
all_sol = [-3.41877   -1.82764  
     -2.88904   1.84693  
     -2.02875   0.00000  
     -1.84693   -2.88904  
     -1.82764   3.41877  
     -1.75560   0.36547 
     -0.36547   -1.7556  
      0.00000   -2.02875  
      0.00000   0.00000  
      0.00000   2.02875  
      0.36547   1.7556  
      1.75560   -0.36547  
      1.82764   -3.41877  
      1.84693   2.88904  
      2.02875   0.00000  
      2.88904   -1.84693  
      3.41877   1.82764 ];
x = all_sol(:, 1);
y = all_sol(:, 2);
% Hessian %
% d^2f/dx^2
H1 = -y .* sin(x + y) - 2 * cos(x - y) + x .* cos(x -y);
% d^2f/dxdy
H2 = cos(x + y) - y .* sin(x + y) + cos(x - y) - x .* sin(x - y);
% d^2f/dydx
H3 = cos(x + y) - y .* sin(x + y) + cos(x - y) - x .* sin(x - y);
% d^2f/dy^2
H4 = x .* sin(x - y) + 2 * cos(x + y) - y .* sin(x + y);
for i = 1:length(x)
    h1 = -y(i) * sin(x(i) + y(i)) - 2 * cos(x(i) - y(i)) + x(i) * cos(x(i) -y(i));
    h2 = cos(x(i) + y(i)) - y(i) * sin(x(i) + y(i)) + cos(x(i) - y(i)) - x(i) * sin(x(i) - y(i));
    h3 = cos(x(i) + y(i)) - y(i) * sin(x(i) + y(i)) + cos(x(i) - y(i)) - x(i) * sin(x(i) - y(i));
    h4 = x(i) * sin(x(i) - y(i)) + 2 * cos(x(i) + y(i)) - y(i) * sin(x(i) + y(i));
    temp_H = [h1 h2;
              h3 h4];
    d = eig(temp_H);
    if all(d > 0)
        H_ans(i) = 1;
    elseif all(d < 0)
        H_ans(i) = -1;
    end
end
z = y .* sin(x + y) - x .* sin(x - y);
figure3 = figure('Name','Mesh Grid With Points','NumberTitle','off');
mesh(X, Y, Z)
xlabel('x')
ylabel('y')
zlabel('z')
hold;
for i = 1:length(x)
    plot3(x(i), y(i), z(i),'.r','markersize',10);
end
figure4 = figure('Name','Contour With Point','NumberTitle','off');
contour(X, Y, Z)
xlabel('x')
ylabel('y')
hold;
for i = 1:length(x)
    plot(x(i), y(i),'.r','markersize',10);
end
print(figure1, '3Dplot.png','-dpng','-r300');
print(figure2, 'Contour.png','-dpng','-r300');
print(figure3, '3DplotWithPoints.png','-dpng','-r300');
print(figure4, 'ContourWithPoints.png','-dpng','-r300');