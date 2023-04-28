%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file : Jacobi.m
% @function : [JacobiMatrix] = Jacobi(coord,l)
% brief : 二轴机械臂雅可比矩阵求解（微分运动学）
% data  : 2021.11.1 
% version : 1.0
% input : theta ------------- 机械臂当前关节角
%         l     ------------- 连杆长度
% output: JacobiMatrix ------------- 雅可比矩阵
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [JacobiMatrix] = Jacobi(theta,l)

JacobiMatrix = [-l(1)*sin(theta(1))-l(2)*sin(theta(1)+theta(2))  -l(2)*sin(theta(1)+theta(2));
                 l(1)*cos(theta(1))+l(2)*cos(theta(1)+theta(2))   l(2)*cos(theta(1)+theta(2))];

end

