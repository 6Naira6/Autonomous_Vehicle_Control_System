clear  %feedback halat
clc
%******************************************
% State Space
A = [-3.785    -19.167   0    0;
   0.469    0.976   0    0;
   0   1   0    0;
   1    0    20    0]

B = [34.409;27.686;0;0]

C = [0 0 0 1]
%******************************************
% poles
sys = ss(A,B,C,0);
tf(sys)

eig_a = eig(A)
pause
%******************************************
% Controllability
phi_c = ctrb(A,B)
rank_c = rank(phi_c)
eig_c = eig(phi_c)

pause
%*****************************************
% Observability
phi_o = obsv(A,C)
rank_o = rank(phi_o)
eig_o = eig(phi_o)

pause
%*****************************************
desired_poles = [-2, -3, -4, -5];
K = acker(A, B, desired_poles)

pause
%*****************************************
% Closed loop system
Acl = A - B*K
Bcl = B
Ccl = C

eig(Acl)