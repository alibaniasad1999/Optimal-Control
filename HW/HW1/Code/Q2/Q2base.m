[X,Y] = meshgrid(linspace(-4, 4));
Z = sin(X) + cos(Y);
Q2_b;
r =sqrt(sol_x^2 + sol_y^2 + sol_z^2);
r = double(r);
[xs, ys, zs] = sphere();
xs = r * xs;
ys = r * ys;
zs = r * zs;
figure1 = figure('Name','Mesh Grid','NumberTitle','off');
plot3(X, Y, Z);
hold;
xlabel('x')
ylabel('y')
zlabel('z')
mesh(xs, ys, zs);
print(figure1, 'plotwithSphere.png','-dpng','-r300');