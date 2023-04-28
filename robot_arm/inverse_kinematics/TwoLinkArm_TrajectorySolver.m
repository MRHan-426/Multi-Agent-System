%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file : TwoLinkArm_TrajectorySolver.m
% brief : Two-axis robotic arm inverse kinematics trajectory solver
% date : 2021.11.1
% version : 1.0
% note : Issues to be addressed
% ② Given a trajectory in the Jacobian space (handwritten letter 'a'), solve the trajectory in the joint space according to the inverse kinematics of the robotic arm,
% and create a robotic arm motion diagram (need to find the position of joint 1, see Robotarm.avi for specific effects, link length in animation is different from the definition below)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all % Clear workspace variables
close all % Close all figures

%% Two-axis robotic arm link length definition
l = [1.1 1.1];

%% Inverse kinematics for a given trajectory
% Read trajectory information
load a1.mat % Trajectory data corresponds to the name 'saveddata', which includes coordinate data x, y, etc., and the trajectory is the handwritten letter 'a'
trajactory_length = size(saveddata.x,2); % Read trajectory length
trajcoord = [saveddata.x',saveddata.y']; % Read trajectory coordinates
trajcoord(:,1) = trajcoord(:,1) - 1; % Change the position of the trajectory for easy robotic arm motion
dt = saveddata.times(2) - saveddata.times(1); % Read time interval

%% Solve the joint space trajectory of the robotic arm inverse kinematics (solve thetaA (including theta1 and theta2)), and create a robotic arm motion diagram (need to find the position of joint 1) <---------
thetaA = zeros(trajactory_length,2); % Initialize theta angles
midtrajA = zeros(trajactory_length,2); % Initialize the position of joint 1

%% Plot
% Generate animation
figure
Robotarm = VideoWriter('Robotarm1.avi'); % Create a new file named 'Robotarm.avi'
open(Robotarm); % Open the 'Robotarm.avi' file
axis([-2 1.3 -1.8 1.5])                                     % Fix the coordinate axis
hold on
plot(trajcoord(:,1),trajcoord(:,2),'r-','linewidth',2);     % Plot the trajectory    
M=moviein(trajactory_length);                               % Moviein needs a plot in front to help with initialization

for k=1:trajactory_length-7
    
    theta = IKrob(trajcoord(k,:),l);

    midtrajA(k,1) = -l(2) * cos(theta(1)+theta(2)) ;
    midtrajA(k,2) = -l(2) * sin(theta(1)+theta(2)) ;        
    
    axis([-2 1.3 -1.8 1.5])
    
    h1 = line([0 midtrajA(k,1)],[0 midtrajA(k,2)],'LineWidth',3);                             % Draw link 1
    h2 = line([midtrajA(k,1) trajcoord(k,1)],[midtrajA(k,2) trajcoord(k,2)],'LineWidth',3);   % Draw link 2
    h3 = plot(midtrajA(k,1),midtrajA(k,2),'bo','LineWidth',6);                                % Draw joint 1

    M(:,k)=getframe;                                        % Capture the figure as a frame of the movie
    writeVideo(Robotarm,M(:,k));
    delete(h1);
    delete(h2);
    delete(h3);
end
% movie(M,1,30); % Play the movie 1 time at 30 frames per second
close(Robotarm); % Close the 'Robotarm.avi' file