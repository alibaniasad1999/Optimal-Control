%%% Quadcopter spacestate %%%
% load data %
QuadConstants;
%{
A1 = sym('A1', 'real');
A3 = sym('A3', 'real');
B1 = sym('B1', 'real');
B3 = sym('B3', 'real');
C2 = sym('C2', 'real');
%}
% dot(x) = Ax + Bu, u = omega^2 matrix
A = [0  0  0 1 0 0  ;
     0  0  0 0 1 0  ;
     0  0  0 0 0 1  ;
     A1 0  0 0 0 0  ;
     0  B1 0 0 0 0  ;
     0  0  0 0 0 0] ;
B = [0   0   0   0  ;
     0   0   0   0  ;
     0   0   0   0  ;
     0   A3  0  -A3 ;
     B3  0  -B3  0  ;
     C2 -C2  C2  C2];