%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file : Q2.m
% brief : 二轴机械臂逆运动学轨迹求解
% data  : 2021.11.1 
% version : 1.0
% note  : 需要解决的问题
%          ② 给定雅可比空间中的一条轨迹(手写字母a)，根据机械臂逆运动学求解关节空间中的轨迹，
%             并作出机械臂运动图（需要求关节1的位置,具体效果见Robotarm.avi，动画中杆长与下面定义不同）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all   % 删除工作区变量
close all   % 关闭所有图

%% 二轴机械臂连杆长度定义
    l = [1.1 1.1];

    
%% Q2 给定轨迹的逆运动学
% 读取轨迹信息
   load a1.mat                                          % 轨迹数据对应名称为saveddata，包含坐标数据x,y等，轨迹为手写字母a的轨迹
   trajactory_length = size(saveddata.x,2);             % 读取轨迹长度
   trajcoord = [saveddata.x',saveddata.y'];             % 读取轨迹坐标
   trajcoord(:,1) = trajcoord(:,1) - 1;                 % 改变一下轨迹的位置，方便机械臂运动
   dt = saveddata.times(2) - saveddata.times(1);        % 读取时间间隔
   
%% 机械臂逆运动学求关节空间轨迹（求解thetaA(包含theta1，theta2)），并作出机械臂运动图（需要求关节1的位置）<---------
    thetaA  = zeros(trajactory_length,2);                   % 初始化theta的角度
    midtrajA = zeros(trajactory_length,2);                  % 初始化关节1的位置
    

%% 画图
% 生成动画
    figure
    Robotarm = VideoWriter('Robotarm1.avi');                     % 新建叫Robotarm.avi的文件
    open(Robotarm);                                             % 打开Robotarm.avi的文件

    axis([-2 1.3 -1.8 1.5])                                     % 固定坐标轴
    hold on
    plot(trajcoord(:,1),trajcoord(:,2),'r-','linewidth',2);     % 画出轨迹    
    M=moviein(trajactory_length);                               % 前面要有plot帮助moviein初始化
    
    for k=1:trajactory_length-7
        
        theta = IKrob(trajcoord(k,:),l);

        midtrajA(k,1) = -l(2) * cos(theta(1)+theta(2)) ;
        midtrajA(k,2) = -l(2) * sin(theta(1)+theta(2)) ;        
        
        axis([-2 1.3 -1.8 1.5])
        
        h1 = line([0 midtrajA(k,1)],[0 midtrajA(k,2)],'LineWidth',3);                             % 画杆1
        h2 = line([midtrajA(k,1) trajcoord(k,1)],[midtrajA(k,2) trajcoord(k,2)],'LineWidth',3);   % 画杆2
        h3 = plot(midtrajA(k,1),midtrajA(k,2),'bo','LineWidth',6);                                % 画关节1

        M(:,k)=getframe;                                        % 抓取图形作为电影的画面
        writeVideo(Robotarm,M(:,k));
        delete(h1);
        delete(h2);
        delete(h3);
    end
%     movie(M,1,30);                                            % 以每秒30帧的速度播放1次
    close(Robotarm);                                            % 关闭
    
 