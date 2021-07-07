%%% Quad in zero LQR solver %%%
QuadSpaceState;
% Weight Matrix %
% syms Q11 Q22 Q33 Q44 Q55 Q66
% Q = [Q11 0   0   0   0   0   ;
%      0   Q22 0   0   0   0   ;
%      0   0   Q33 0   0   0   ;
%      0   0   0   Q44 0   0   ;
%      0   0   0   0   Q55 0   ;
%      0   0   0   0   0   Q66];
% syms R11 R22 R33 R44
% R = [R11 0   0   0   ;
%      0   R22 0   0   ;
%      0   0   R33 0   ;
%      0   0   0   R44];
% Riccati Solver %
Q = eye(6);
R = eye(4);
% algebraic_riccati = K * A + A' * K - K * B * R^-1 * B' * K + Q