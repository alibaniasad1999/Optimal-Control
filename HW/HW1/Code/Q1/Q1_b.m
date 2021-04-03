[X,Y] = meshgrid(linspace(-4, 4));
Z = X.^3 - 3 * X .* Y .^ 2;
figure1 = figure('Name','Mesh Grid','NumberTitle','off');
mesh(X, Y, Z)
xlabel('x')
ylabel('y')
zlabel('z')
print(figure1, '3DplotQ1b.png','-dpng','-r300');
figure2 = figure('Name','Mesh Grid with point','NumberTitle','off');
mesh(X, Y, Z)
hold;
xlabel('x')
ylabel('y')
zlabel('z')
plot3(0, 0, 0,'.r','markersize',10);
print(figure2, '3DplotWithPointsQ1b.png','-dpng','-r300');
figure3 = figure('Name','Contour','NumberTitle','off');
contour(X, Y, Z)
xlabel('x')
ylabel('y')
figure4 = figure('Name','Contour With Point','NumberTitle','off');
contour(X, Y, Z)
xlabel('x')
ylabel('y')
hold;
plot(0, 0,'.r','markersize',10);
print(figure3, 'ContourQ1b.png','-dpng','-r300');
print(figure4, 'ContourWithPointsQ1b.png','-dpng','-r300');