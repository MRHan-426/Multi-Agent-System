%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file : IKrob.m
% @function : [theta] = IKrob(coord,l)
% brief : 二轴机械臂逆运动学求解函数
% data  : 2021.11.1 
% version : 1.0
% input : coord ------------- 笛卡尔空间坐标
%         l     ------------- 连杆长度
% output: theta ------------- 机械臂关节角
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [theta] = IKrob(coord,l)
x=coord(1);
y=coord(2);
L_1=l(1);
% L_2=l(2);

theta2 = -2*acos(sqrt(x^2+y^2)/(2*L_1)); %正负问题
theta1 = atan(y/x)-theta2/2;

theta = [theta1 theta2];
end