%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file : TwoLinkArm_TrajectorySolver.m
% brief : Two-link robotic arm inverse kinematics trajectory solution
% data : 2021.11.1
% version : 1.0
% note : Problem to be solved
% ② Given a trajectory in the Jacobian space (handwritten letter 'a'), find the trajectory in joint space using the inverse kinematics of the robotic arm,
% and create a motion diagram of the robotic arm (need to calculate the position of joint 1, see Robotarm.avi for specific effect, link lengths in the animation are different from the definition below)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; % Clear workspace variables
close all; % Close all figures

%% Two-link robotic arm link length definition
l = [1.1, 1.1];

%% Q2 Inverse kinematics for a given trajectory
% Load trajectory information
load a1.mat; % Trajectory data corresponds to the name "saveddata", including coordinate data x, y, etc., the trajectory is the handwritten letter 'a'
trajactory_length = size(saveddata.x, 2); % Read trajectory length
trajcoord = [saveddata.x', saveddata.y']; % Read trajectory coordinates
trajcoord(:, 1) = trajcoord(:, 1) - 1; % Change the position of the trajectory to facilitate robotic arm movement
dt = saveddata.times(2) - saveddata.times(1); % Read time interval

%% Inverse kinematics to find joint space trajectory (solving thetaA (including theta1, theta2)) and create a motion diagram of the robotic arm (need to calculate the position of joint 1) <---------
thetaA = zeros(trajactory_length, 2); % Initialize theta angles
midtrajA = zeros(trajactory_length, 2); % Initialize the position of joint 1

%% Plotting
% Generate animation
figure;
Robotarm = VideoWriter('Robotarm1.avi'); % Create a new file called Robotarm.avi
open(Robotarm); % Open Robotarm.avi file
axis([-2, 1.3, -1.8, 1.5]);                              % Fix coordinate axis
hold on;
plot(trajcoord(:, 1), trajcoord(:, 2), 'r-', 'linewidth', 2); % Draw trajectory
M = moviein(trajactory_length);                          % A plot is needed in the front to help moviein initialize

for k = 1:trajactory_length - 7

    theta = IKrob(trajcoord(k, :), l);

    midtrajA(k, 1) = -l(2) * cos(theta(1) + theta(2));
    midtrajA(k, 2) = -l(2) * sin(theta(1) + theta(2));

    axis([-2, 1.3, -1.8, 1.5]);

    h1 = line([0, midtrajA(k, 1)], [0, midtrajA(k, 2)], 'LineWidth', 3);                             % Draw link 1
    h2 = line([midtrajA(k, 1), trajcoord(k, 1)], [midtrajA(k, 2), trajcoord(k, 2)], 'LineWidth', 3); % Draw link 2
    h3 = plot(midtrajA(k, 1), midtrajA(k, 2), 'bo', 'LineWidth', 6); % Draw joint 1
    M(:, k) = getframe;                                       % Capture the figure as a movie frame
    writeVideo(Robotarm, M(:, k));
    delete(h1);
    delete(h2);
    delete(h3);
end
% movie(M, 1, 30); % Play the movie at 30 frames per second for 1 time
close(Robotarm); % Close Robotarm.avi file
