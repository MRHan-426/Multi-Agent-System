%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file : TwoLinkArm_InverseKinematics.m
% brief : Two-axis robotic arm inverse kinematics solver
% date : 2021.11.1
% version : 1.0
% note : Issues to be addressed
% ① Given a coordinate point in the Jacobian space, solve the corresponding joint angles according to the inverse kinematics of the robotic arm (complete IKrob.m)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all % Clear workspace variables
close all % Close all figures

%% Two-axis robotic arm link length definition
l = [0.7 1];

%% Inverse kinematics for a given point
dot = [0.8 0.8]; % Specify the point the robotic arm needs to reach
theta = IKrob(dot, l); % Solve the corresponding joint angles <------------------
figure                                                                 % Plot
axis equal
hold on
plot(dot(1),dot(2),'r*','LineWidth',10);                               % Plot target point
midtraj = [l(1) * cos(theta(1)),l(1) * sin(theta(1))];                 % Calculate the position of joint 1

h1 = line([0 midtraj(1)],[0 midtraj(2)],'LineWidth',3);                % Draw link 1
h2 = line([midtraj(1) dot(1)],[midtraj(2) dot(2)],'LineWidth',3);      % Draw link 2
plot(midtraj(1),midtraj(2),'bo','LineWidth',6);                        % Draw joint 1
