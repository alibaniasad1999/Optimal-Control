[X,Y] = meshgrid(linspace(-4, 4));
Z = X.^3 - 3 * X .* Y .^ 2;
figure1 = figure('Name','Mesh Grid','NumberTitle','off');
mesh(X, Y, Z)
xlabel('x')
ylabel('y')
zlabel('z')
print(figure1, '3DplotQ1b.png','-dpng','-r300');