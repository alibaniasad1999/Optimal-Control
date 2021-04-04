[X,Y] = meshgrid(linspace(-4, 4));
Z = sin(X) + cos(Y);
mesh(X, Y, Z);