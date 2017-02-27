%% Deriving the Inverse Dynamics %%
clc;
clear all;

%symbolic variables
syms m M l I t g u;
syms x dx ddx theta dtheta ddtheta;

%state of pole COM
xPole = x + l*sin(theta);
yPole = -l*cos(theta);
dxPole = dx + l*cos(theta)*dtheta;
dyPole = l*sin(theta)*dtheta;

%% Lagrangian %%
%kinetic energy
Tcart = M*dx^2 / 2;
Tpole = (m*((dxPole^2) + (dyPole^2))) / 2;
T = Tcart + Tpole;

%potential energy
Ucart = 0;
Upole = -m*g*yPole;
U = Ucart + Upole;

%lagrangian
L = simplify(T-U);

%inverse dynamics
q = [x; theta]; %generalized coordinates
dq = [dx; dtheta];
ddq = [ddx; ddtheta];
F = [u;0];

dLddq = jacobian(L,dq);
dLdq = jacobian(L,q);
ddtdLddq = jacobian(dLddq,dq)*ddq + jacobian(dLddq,q)*dq;

%Equations of Motion
EOM = ddtdLddq - transpose(dLdq) - F;
solution = solve(EOM,ddq);

ddxSolution = simplify(solution.ddx);
ddthetaSolution = simplify(solution.ddtheta);
% ddxSolution = subs(ddxSolution,'m',0.1);
% ddxSolution = subs(ddxSolution,'M',1);
% ddxSolution = subs(ddxSolution,'l',1);
% ddxSolution = subs(ddxSolution,'g',-9.81);
% ddthetaSolution = subs(ddthetaSolution,'m',0.1);
% ddthetaSolution = subs(ddthetaSolution,'M',1);
% ddthetaSolution = subs(ddthetaSolution,'l',1);
% ddthetaSolution = subs(ddthetaSolution,'g',-9.81);

%Standard Form
H = [M+m l*m*cos(theta); l*m*cos(theta) l^2*m];
C = [0 -l*m*sin(theta)*dtheta; 0 0];
G = [0; -g*l*m*sin(theta)];
B = [1;0];







