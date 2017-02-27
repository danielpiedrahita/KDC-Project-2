clc;
clear;

%Add Necessary Files to Path
currentPath = path();
addpath((currentPath));

%Put into state space form
A = [0 1 0      0; ...
     0 0 0.7164  0;...
     0 0 0      1; ...
     0 0 15.76 0];
B = [0;0.9755;0;1.46];
C = eye(4);
D = 0;
sys = ss(A,B,C,D);

%Check controllability
if rank(ctrb(sys))==4
    disp('System is controllable!');
else
    disp('System is not controllable :(');
end

%LQR Controller
Q = [25 0 0 0; 0 1 0 0; 0 0 10 0; 0 0 0 1]; %cost on states
R = 10; %cost on inputs
[K,S,E] = lqr(sys,Q,R);

% Trajectory Optimization

%Setup
p.m1 = 1.0;  % (kg) Cart mass
p.m2 = 0.1;  % (kg) pole mass
p.g = -9.81;  % (m/s^2) gravity
p.l = 1;   % (m) pendulum (pole) length

dist = 1.5;  %How far must the cart translate during its swing-up
maxForce = 200;  %Maximum actuator forces
duration = 1.9; %1.4 nice for 1 swing swing-up, 1.9 for 2 swing, 2.25 for 3 swing

problem.func.dynamics = @(t,x,u)( cartPoleDynamics(x,u,p) );
problem.func.pathObj = @(t,x,u)( u.^2 );  %Input-squared cost function

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = duration;
problem.bounds.finalTime.upp = duration;

problem.bounds.initialState.low = [dist;pi;0;0];
problem.bounds.initialState.upp = [dist;pi;0;0];
problem.bounds.finalState.low = zeros(4,1);
problem.bounds.finalState.upp = zeros(4,1);

problem.bounds.state.low = [-2*dist;-2*pi;-inf;-inf];
problem.bounds.state.upp = [2*dist;2*pi;inf;inf];

problem.bounds.control.low = -maxForce;
problem.bounds.control.upp = maxForce;

problem.guess.time = [0,duration];
problem.guess.state = [problem.bounds.initialState.low, problem.bounds.finalState.low];
problem.guess.control = [0,0];

problem.options.nlpOpt = optimset(...
    'Display','iter',...
    'MaxFunEvals',1e5);

problem.options.method = 'hermiteSimpson';

%Solve
soln = optimTraj(problem);

%%% Unpack the solution
t = linspace(soln.grid.time(1), soln.grid.time(end), 150);
z = soln.interp.state(t);
u = soln.interp.control(t);
trajectory = [t' u'];

%Run Simulation
sim('part6sim.slx');

% %symbolic variables
% syms m M l t g u;
% syms x dx ddx theta dtheta ddtheta;
% 
% l = 1;
% m = 0.1;
% M = 1;
% I = (m*l^2)/12;
% 
% %state of pole COM
% xPole = x + l*sin(theta);
% yPole = l*cos(theta);
% dxPole = dx + l*cos(theta)*dtheta;
% dyPole = -l*sin(theta)*dtheta;
% 
% %% Lagrangian %%
% %kinetic energy
% Tcart = M*dx^2 / 2;
% Tpole = ((m*((dxPole^2) + (dyPole^2))) + (I*dtheta^2)) / 2;
% T = Tcart + Tpole;
% 
% %potential energy
% Ucart = 0;
% Upole = m*g*yPole;
% U = Ucart + Upole;
% 
% %lagrangian
% L = simplify(T-U);
% 
% %inverse dynamics
% q = [x; theta]; %generalized coordinates
% dq = [dx; dtheta];
% ddq = [ddx; ddtheta];
% F = [u;0];
% 
% X = [x;theta;dx;dtheta]; %states
% dX = [dx;dtheta;ddx;ddtheta];
% 
% dLddq = jacobian(L,dq);
% dLdq = jacobian(L,q);
% ddtdLddq = jacobian(dLddq,dq)*ddq + jacobian(dLddq,q)*dq;
% 
% %Equations of Motion
% EOM = ddtdLddq - transpose(dLdq) - F;
% solution = solve(EOM,ddq);
% ddxSolution = simplify(solution.ddx);
% ddthetaSolution = simplify(solution.ddtheta);
% 
% %Linearize around unstable point
% ddxLinear = taylor(ddxSolution,X,'Order',2);
% ddthetaLinear = taylor(ddthetaSolution,X,'Order',2);
% 
% ddxLinear = subs(ddxLinear,'m',0.1);
% ddxLinear = subs(ddxLinear,'M',1);
% ddxLinear = subs(ddxLinear,'l',1);
% ddxLinear = subs(ddxLinear,'g',-9.81);
% ddthetaLinear = subs(ddthetaLinear,'m',0.1);
% ddthetaLinear = subs(ddthetaLinear,'M',1);
% ddthetaLinear = subs(ddthetaLinear,'l',1);
% ddthetaLinear = subs(ddthetaLinear,'g',-9.81);








