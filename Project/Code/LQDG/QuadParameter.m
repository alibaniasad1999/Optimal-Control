% Distance between rotor center to quad center (m)
d_cg = 0.295;
% Distance between D point to quad center (m)
h_cg = 0.0078;
% Thrust factor (non dimensional)
b = 1.10794e-5;
% Rotor torque drag factor (non dimensional)
d = 3.2e-6;
% Total mass (kg)
m_tot = 1.774;
% Quad Moment of inertia respect to axix X Body (kg.m^2)
J_xx = 0.0297598;
% Quad Moment of inertia respect to axix Y Body (kg.m^2)
J_yy = 0.0298565;
% Quad Moment of inertia respect to axix Y Body (kg.m^2)
J_zz = 0.058510;
% Quad blade Moment (kg.m^2)
J_R = 4.439e-5;
% Mass on X axis bearing (kg)
m1 = 1.272;
% Mass on Y axis bearing (kg)
m2 = 1.074;
% Mass on Z axis bearing (kg)
m3 = 1.693;
% Bearing radius in X axix (m)
r_x = 0.01;
% Bearing radius in Y axix (m)
r_y = 0.01;
% Bearing radius in Z axix (m)
r_z = 0.025;
% Static bearing friction (non dimensional)
miu_s = 0.003;
% Dynamic bearing friction (non dimensional)
miu_k = 0.002;
% Motor constant (non dimensional)
tau = 0.15;
% Delay of motor (sec)
T_d = 0.02;
% Gravity force (m/s^2)
g = 9.81;