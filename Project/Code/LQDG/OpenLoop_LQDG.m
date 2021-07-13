global A B Q n S1 S2 R1_inv u_saver counter t_u
[A, B] = Quadcopter_system(zeros(6, 1), ones(4, 1) * 2000);
Q      = 100*eye(6);
R1     = eye(4);
R1_inv = R1^-1;
R2     = 2 * eye(4);
R2_inv = R2^-1;
S1     = B* R1_inv * B';
S2     = B* R2_inv * B';
H	   = 100*eye(6);
tf     = 10;
p0	= [H;H];
n	= 6;
p0	= reshape(p0,2 * n^2,1);
global t_p p_arr
[t_p,p_arr] = ode45(@diff_eq_Riccati,[tf,0],p0);
for i = 1:7
    u_saver = zeros(4,1);
    t_u = zeros(1);
    counter = 1;
    if i == 7
        x0 = ones(6, 1);
    else
        x0 = zeros(6, 1);
        x0(i) = 1;
    end
    [t,x] = ode45(@diff_eq_states,[0,tf],x0);
    figure(101)
    plot(t, x)
    legend('$\phi$', '$\theta$', '$\psi$','$p$','$q$','$r$',...
        'interpreter', 'latex')
    xlabel('$Time_{(\sec)}$', 'interpreter', 'latex');
    ylabel('$System~State$', 'interpreter', 'latex');
    figure(102)
    plot(t_u, 2e6*u_saver);
    legend('$\omega_1$', '$\omega_2$', '$\omega_3$','$\omega_4$',...
        'interpreter', 'latex')
    xlabel('$Time_{(\sec)}$', 'interpreter', 'latex');
    ylabel('$RPM$', 'interpreter', 'latex');
    switch i
        case 1
            print(101, '../../Figure/LQDG/OpenLoopLQDGroll.png','-dpng','-r500')
            print(102, '../../Figure/LQDG/OpenLoopLQDGrollcontrol.png','-dpng','-r500')
        case 2
            print(101, '../../Figure/LQDG/OpenLoopLQDGpitch.png','-dpng','-r500')
            print(102, '../../Figure/LQDG/OpenLoopLQDGpitchcontrol.png','-dpng','-r500')
        case 3
            print(101, '../../Figure/LQDG/OpenLoopLQDGpsi.png','-dpng','-r500')
            print(102, '../../Figure/LQDG/OpenLoopLQDGpsicontrol.png','-dpng','-r500')
        case 4
            print(101, '../../Figure/LQDG/OpenLoopLQDGp.png','-dpng','-r500')
            print(102, '../../Figure/LQDG/OpenLoopLQDGpcontrol.png','-dpng','-r500')
        case 5
            print(101, '../../Figure/LQDG/OpenLoopLQDGq.png','-dpng','-r500')
            print(102, '../../Figure/LQDG/OpenLoopLQDGqcontrol.png','-dpng','-r500')
        case 6
            print(101, '../../Figure/LQDG/OpenLoopLQDGr.png','-dpng','-r500')
            print(102, '../../Figure/LQDG/OpenLoopLQDGrcontrol.png','-dpng','-r500')
        otherwise
            print(101, '../../Figure/LQDG/OpenLoopLQDGall.png','-dpng','-r500')
            print(102, '../../Figure/LQDG/OpenLoopLQDGallcontrol.png','-dpng','-r500')
    end
%     close all;
end

% x(:, 1:3) = wrapToPi(x(:, 1:3));

%% Functions %%
function d = diff_eq_Riccati(~,p)
global A Q n S1 S2
p	= reshape(p,2 * n,n);
p1 = p(1:n, :);
p2 = p(n+1: end, :);
p1dot	= -p1*A - A'*p1 - Q +  p1*S1*p1 + p1 * S2 * p2;
p2dot	= -p2*A - A'*p2 - Q +  p2*S2*p2 + p2 * S1 * p1;
d	= reshape([p1dot;p2dot],2 * n^2,1);
end

function d = diff_eq_states(t,x)
global n u R1_inv u_saver counter t_u
global t_p p_arr S1 S2
if t == 0
    u = zeros(4, 1);
end
t_u(counter) = t;
[A, B] = Quadcopter_system(x, u);
p_t	= interp1(t_p, p_arr, t);
p	= reshape(p_t,2 * n,n);
p1 = p(1:n, :);
p2 = p(n+1: end, :);
d	= (A - S1 * p1 - S2 * p2) * x;
u = R1_inv * B' * p1 * x;
u_saver(:, counter) = u;
counter = counter + 1;
end