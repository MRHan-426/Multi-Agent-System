%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file : TwoLinkArm_JacobianInverseSolution.m
% brief : Two-axis robotic arm inverse kinematics and Jacobian matrix solver
% data : 2021.11.1
% version : 1.0
% note : Problems to solve:
% ¢Û Given a trajectory in the Jacobian space (handwritten letter 'a'),
% solve for the trajectory in the joint space using the inverse Jacobian matrix (edit Jacobi.m),
% and create a robotic arm motion diagram (calculate the position of joint 1,
% specific effects can be seen in Robotarm.avi, the rod length in the animation is different from the definition below)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%clear all % Clear workspace variables
close all % Close all figures

%% Define the length of the two-axis robotic arm links
l = [1.1 1.1];

%% Inverse Jacobian matrix to find joint space trajectory
% Read trajectory information
load a1.mat % Trajectory data corresponding to the name 'saveddata', containing coordinate data x, y, etc., the trajectory is the handwritten letter 'a' trajectory
trajactory_length = size(saveddata.x,2); % Read the trajectory length
trajcoord = [saveddata.x',saveddata.y']; % Read the trajectory coordinates
trajspeed = [saveddata.vx',saveddata.vy']; % Read the trajectory speed
dt = saveddata.times(2) - saveddata.times(1); % Read the time interval
trajcoord(:,1) = trajcoord(:,1) - 1; % Change the trajectory position to facilitate robotic arm movement

%% Inverse Jacobian matrix solver (you can first use inverse kinematics to find the initial pose of the robotic arm) <----------------
thetaB  = zeros(trajactory_length,2);                    % Initialize robotic arm joint angles
midtrajB = zeros(trajactory_length,2);                   % Initialize the position of joint 1
% thetaB_dot=zeros(trajactory_length,2); % Initialize robotic arm joint angle speeds

%% Plot
% Create animation
figure
Robotarm = VideoWriter('Robotarm2.avi'); % Create a new file called 'Robotarm.avi'
open(Robotarm); % Open 'Robotarm.avi' file
axis([-2 1.3 -1.8 1.5])                                     % Fix the coordinate axis
hold on
plot(trajcoord(:,1),trajcoord(:,2),'r-','linewidth',2);     % Plot the trajectory    

M=moviein(trajactory_length);                               % Use plot to help initialize moviein

for k=1:trajactory_length
    thetaB(k,:) = IKrob(trajcoord(k,:),l);
    if k > 1
        Jacobian_Matrix = Jacobi(thetaB(k,:),l);
        
        thetaBspeed = Jacobian_Matrix^(-1) * trajspeed(k,:)';
        
        thetaB(k,:) = thetaB(k-1,:) - thetaBspeed' * dt;
    end
    midtrajB(k,1) = -l(2) * cos(thetaB(k,1)+thetaB(k,2)) ;
    midtrajB(k,2) = -l(2) * sin(thetaB(k,1)+thetaB(k,2)) ;
    axis([-2 1.3 -1.8 1.5])
    h1 = line([0 midtrajB(k,1)],[0 midtrajB(k,2)],'LineWidth',3); % Draw link 1
    h2 = line([midtrajB(k,1) trajcoord(k,1)],[midtrajB(k,2) trajcoord(k,2)],'LineWidth',3); % Draw link 2
    h3 = plot(midtrajB(k,1),midtrajB(k,2),'bo','LineWidth',6); % Draw joint 1
    M(:,k)=getframe;                                        % Capture the figure as a movie frame
    writeVideo(Robotarm,M(:,k));
    delete(h1);
    delete(h2);
    delete(h3);
end
% movie(M,1,50); % Play the movie at 50 frames per second for 1 time
close(Robotarm); % Close the 'Robotarm.avi' file