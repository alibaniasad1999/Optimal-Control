[X,Y] = meshgrid(linspace(-4, 4));
Z = Y .* sin(X + Y) - X .* sin(X - Y);
figure('Name','Mesh Grid','NumberTitle','off');
mesh(X, Y, Z)
figure('Name','Contour','NumberTitle','off');
contour(X, Y, Z)