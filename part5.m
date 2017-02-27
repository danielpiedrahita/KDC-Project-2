clc;
clear;

%%Build LQR Tree Lookup
load Linearization.mat;
for i = 1:9 
    A = LinearAnalysisToolProject.Results.Data.Value(:,:,i,1).A;
    B = LinearAnalysisToolProject.Results.Data.Value(:,:,i,1).B;
    C = eye(4);
    D = 0;
    sys = ss(A,B,C,D);
    
    %Check controllability
    if rank(ctrb(sys))==4
        %disp('System is controllable!');
    else
        disp('System is not controllable :(');
    end

    %LQR Controller
    Q = [25 0 0 0; ...
        0 5 0 0; ...
        0 0 10 0; ...
        0 0 0 1]; %cost on states
    R = 2; %cost on inputs
    [K,S,E] = lqr(sys,Q,R);
    K_lookup(i,:) = K;
end
% K = [-0.5 -1 25 10];

%Simulation Parameters
theta_initial =  75;
dtheta_initial = 0;

%Run Simulation
sim('part5sim.slx');

% %Basin of Attraction Analysis
% numThetas = 45;
% numDThetas = 25;
% theta_initials = linspace(0,90,numThetas+1);
% dtheta_initials = linspace(0,1900,numDThetas+1);
% counter = 0;
% dataMatrix = [0 0 1];
% for i=1:numThetas
%     for j=1:numDThetas
%         counter = counter + 1;
%         theta_initial = theta_initials(i); %[deg]
%         dtheta_initial = dtheta_initials(j); %[deg/s]
%         tic;
%         sim('part5sim.slx');        
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


% dataMatrixPart4 = load('part4BasinOfAttraction.mat');
% dataMatrixPart4 = dataMatrixPart4.dataMatrix;
% 
% dataMatrixPart5 = load('part5BasinOfAttraction.mat');
% dataMatrixPart5 = dataMatrixPart5.dataMatrix;
% 
% 
% %%Fill out results
% dataMatrixFull4 = [dataMatrixPart4; ...
%     [-dataMatrixPart4(:,1) dataMatrixPart4(:,2) dataMatrixPart4(:,3)];...
%     [dataMatrixPart4(:,1) -dataMatrixPart4(:,2) dataMatrixPart4(:,3)];...
%     [-dataMatrixPart4(:,1) -dataMatrixPart4(:,2) dataMatrixPart4(:,3)]];
% 
% dataMatrixFull5 = [dataMatrixPart5; ...
%     [-dataMatrixPart5(:,1) dataMatrixPart5(:,2) dataMatrixPart5(:,3)];...
%     [dataMatrixPart5(:,1) -dataMatrixPart5(:,2) dataMatrixPart5(:,3)];...
%     [-dataMatrixPart5(:,1) -dataMatrixPart5(:,2) dataMatrixPart5(:,3)]];
% 
% %% Plot Results
% figure(1);
% hold on;
% for i=1:length(dataMatrixFull4)
%     if (dataMatrixFull4(i,3)==1)
%         plot(dataMatrixFull4(i,1), dataMatrixFull4(i,2), 'r*');
%     end
% end
% for i=1:length(dataMatrixFull5)
%     if (dataMatrixFull5(i,3)==1)
%         plot(dataMatrixFull5(i,1), dataMatrixFull5(i,2), 'b*');
%     end
% end
% hold off;
% xlabel('Theta [deg]');
% ylabel('dTheta [deg/s]');
% title('Basin of Attraction Comparison');

