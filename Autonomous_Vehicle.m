clc
clear

A = [-3.785    -19.167   0    0;
   0.469    0.976   0    0;
   0   1   0    0;
   1    0    20    0]

B = [34.409;27.686;0;0]

C = [0 0 0 1]
%******************************************
% poles
sys = ss(A,B,C,0);
H = tf(sys)

eig_a = eig(A)
%******************************************
% Controllability
phi_c = ctrb(A,B)
rank_c = rank(phi_c)
%*****************************************
% Observability
phi_o = obsv(A,C)
rank_o = rank(phi_o)
%*****************************************
% Controllable Canonical Form
sys = ss(A,B,C,0);
csys = canon(sys,'companion')
%******************************************
% Observable Canonical Form
Hccom = canon(H,"companion");
Hocom = ss ;
Hocom.A = Hccom.A'
Hocom.B = Hccom.C'
Hocom.C = Hccom.B'
Hocom.D = Hccom.D'
%*****************************************
% State feedback controller
P_SF = [-2, -3, -4, -5];
K = acker(A, B, P_SF)

P_o = [-10, -11, -12, -13]
L = acker(A', C', P_o)'
%*******************************************
% Closed-loop system
Acl = [A   -B*K; L*C A-L*C-B*K]
Bcl = [B; zeros(4,1)]
Ccl = eye(8)

X0 = [1, 2, 3, 4, 0, 0, 0, 0];
SYScl = ss(Acl, Bcl, Ccl, 0)
t = 0:0.001:5;
u = 0 * ones(1,length(t));

[Y,T,X] = lsim(SYScl, u, t, X0);
%*******************************************
%ploting the results
subplot(2,2,1)
plot(t, X(:,1),'b')
hold on
plot(t, X(:,5),'k--')

subplot(2,2,2)
plot(t, X(:,2),'b')
hold on
plot(t, X(:,6),'k--')

subplot(2,2,3)
plot(t, X(:,3),'b')
hold on
plot(t, X(:,7),'k--')

subplot(2,2,4)
plot(t, X(:,4),'b')
hold on
plot(t, X(:,8),'k--')