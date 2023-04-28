%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file : Jacobi.m
% @function : [JacobiMatrix] = Jacobi(coord,l)
% brief : Two-axis robotic arm Jacobian matrix solver (differential kinematics)
% date : 2021.11.1
% version : 1.0
% input : theta ------------- Current joint angles of the robotic arm
% l ------------- Link lengths
% output: JacobiMatrix ------------- Jacobian matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [JacobiMatrix] = Jacobi(theta,l)

JacobiMatrix = [-l(1)*sin(theta(1))-l(2)*sin(theta(1)+theta(2)) -l(2)*sin(theta(1)+theta(2));
l(1)*cos(theta(1))+l(2)*cos(theta(1)+theta(2)) l(2)*cos(theta(1)+theta(2))];

end