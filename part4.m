clc;
clear;

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
Q = [10 0 0 0; 0 1 0 0; 0 0 10 0; 0 0 0 1]; %cost on states
R = 10; %cost on inputs
[K,S,E] = lqr(sys,Q,R);

%Simulation Parameters
theta_initial = 45;
dtheta_initial = 0;

%Run Simulation
sim('part3sim.slx');

% %Basin of Attraction Analysis
% numThetas = 45;
% numDThetas = 25;
% theta_initials = linspace(0,90,numThetas+1);
% dtheta_initials = linspace(0,500,numDThetas+1);
% counter = 0;
% dataMatrix = [0 0 1];
% for i=1:numThetas
%     for j=1:numDThetas
%         counter = counter + 1;
%         theta_initial = theta_initials(i); %[deg]
%         dtheta_initial = dtheta_initials(j); %[deg/s]
%         tic;
%         sim('part4sim.slx');        
%         if (isempty(endTime.Time))
%             success = 1;
%         else
%             success = 0;
%         end
%         dataMatrix = [dataMatrix; theta_initial dtheta_initial success];
%         disp('Iteration: ');
%         disp(counter);
%         disp('Percent Done:');
%         disp(100*(counter/(numThetas*numDThetas + 2)));
%         disp('Success: ');
%         disp(success);
%         disp('Sim Time');
%         disp(toc);
%     end
% end

% dataMatrix = load('part4BasinOfAttraction.mat');
% dataMatrix = dataMatrix.dataMatrix;
% 
% 
% %%Fill out results
% dataMatrixFull = [dataMatrix; ...
%     [-dataMatrix(:,1) dataMatrix(:,2) dataMatrix(:,3)];...
%     [dataMatrix(:,1) -dataMatrix(:,2) dataMatrix(:,3)];...
%     [-dataMatrix(:,1) -dataMatrix(:,2) dataMatrix(:,3)]];
% 
% %% Plot Results
% figure(1);
% hold on;
% for i=1:length(dataMatrixFull)
%     if (dataMatrixFull(i,3)==1)
%         plot(dataMatrixFull(i,1), dataMatrixFull(i,2), 'b*');
%     end
% end
% hold off;
% xlabel('Theta [deg]');
% ylabel('dTheta [deg/s]');
% title('Basin of Attraction');





