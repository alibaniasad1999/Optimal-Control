%%% ploter %%%
%% u = 1%%
figurepu = figure('Name', 'u(t) = 1', 'NumberTitle','off');
hold on;
C5 = [logspace(-1, 1, 3), -logspace(-1, 1, 3)];
x1 = -2:2;
for i = 1:length(C5)
    x = x1;
    y = x1*C5(i) - C5(i) + 1;
    plot(x, y);
end
axis([-2 2 -2 2]);
legend('$C_5 = 0.1 $', '$C_5 = 1.0$', '$C_5 = 10$', '$C_5 = -0.1$', '$C_5 = -1.0$', '$C_6 = -10$'...
    , 'Interpreter','latex');
xlabel('$x_1(t)$', 'Interpreter','latex');
ylabel('$x_2(t)$', 'Interpreter','latex');
print(figurepu, 'figures/lowu1x1x2.png', '-dpng','-r400');
close;
%%% new color figure %%%
figurepu = figure('Name', 'u(t) = 1 switch curve', 'NumberTitle','off');
hold on;
x1 = linspace(-4, 4); % better view
for i = 1:length(C5)
    x = x1;
    y = x1*C5(i) - C5(i) + 1;
    if C5(i) == 1
        plot(x, y, 'r', 'linewidth', 2);
         j = 1;
    while 1
         if j > length(x)
            break
         end
        quiver(x(j), y(j), -(x(j) - 1) , -(y(j) - 1), 0.7, 'r');
        j = j + 10;
    end
    else
        plot(x, y, 'k');
            j = 1;
    while 1
         if j > length(x)
            break
         end
        quiver(x(j), y(j), -(x(j) - 1) , -(y(j) - 1), 0.5, 'k');
        j = j + 10;
    end
    end
end
axis([-2 2 -2 2]);
xlabel('$x_1(t)$', 'Interpreter','latex');
ylabel('$x_2(t)$', 'Interpreter','latex');
print(figurepu, 'figures/lowu1x1x2SC.png', '-dpng','-r400');
close;
%% u = -1%%
figurepu = figure('Name', 'u(t) = -1', 'NumberTitle','off');
hold on;
C5 = [logspace(-1, 1, 3), -logspace(-1, 1, 3)];
x1 = -2:2;
for i = 1:length(C5)
    x = x1;
    y = x1*C5(i) + C5(i) - 1;
    plot(x, y);
end
axis([-2 2 -2 2]);
legend('$C_5 = 0.1 $', '$C_5 = 1.0$', '$C_5 = 10$', '$C_5 = -0.1$', '$C_5 = -1.0$', '$C_6 = -10$'...
    , 'Interpreter','latex');
xlabel('$x_1(t)$', 'Interpreter','latex');
ylabel('$x_2(t)$', 'Interpreter','latex');
print(figurepu, 'figures/lowum1x1x2.png', '-dpng','-r400');
close;
%%% new color figure %%%
figurepu = figure('Name', 'u(t) = -1 switch curve', 'NumberTitle','off');
hold on;
x1 = linspace(-4, 4); % better view
for i = 1:length(C5)
    x = x1;
    y = x1*C5(i) + C5(i) - 1;
    if C5(i) == 1
        plot(x, y, 'r', 'linewidth', 2);
         j = 1;
    while 1
         if j > length(x)
            break
         end
        quiver(x(j), y(j), -(x(j) + 1) , -(y(j) + 1), 0.7, 'r');
        j = j + 10;
    end
    else
        plot(x, y, 'k');
            j = 1;
    while 1
         if j > length(x)
            break
         end
        quiver(x(j), y(j), -(x(j) + 1) , -(y(j) + 1), 0.5, 'k');
        j = j + 10;
    end
    end
end
axis([-2 2 -2 2]);
xlabel('$x_1(t)$', 'Interpreter','latex');
ylabel('$x_2(t)$', 'Interpreter','latex');
print(figurepu, 'figures/lowum1x1x2SC.png', '-dpng','-r400');
close;