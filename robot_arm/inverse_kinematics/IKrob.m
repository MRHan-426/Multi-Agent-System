%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file : IKrob.m
% @function : [theta] = IKrob(coord,l)
% brief : Two-axis robotic arm inverse kinematics solver function
% date : 2021.11.1
% version : 1.0
% input : coord ------------- Cartesian space coordinates
% l ------------- Link lengths
% output: theta ------------- Robotic arm joint angles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [theta] = IKrob(coord,l)
x=coord(1);
y=coord(2);
L_1=l(1);
% L_2=l(2);

theta2 = -2acos(sqrt(x^2+y^2)/(2L_1)); %Positive or negative issue
theta1 = atan(y/x)-theta2/2;

theta = [theta1 theta2];
end