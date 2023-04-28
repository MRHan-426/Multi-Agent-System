%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @file : Q3.m
% brief : �����е�����˶�ѧ�ſɱȾ������
% data  : 2021.11.1 
% version : 1.0
% note  : ��Ҫ���������
%          �� �����ſɱȿռ��е�һ���켣(��д��ĸa)���������ſɱȾ������ؽڿռ��еĹ켣����дJacobi.m����
%             ��������е���˶�ͼ����Ҫ��ؽ�1��λ��,����Ч����Robotarm.avi�������и˳������涨�岻ͬ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%clear all   % ɾ������������
close all   % �ر�����ͼ

%% �����е�����˳��ȶ���
    l = [1.1 1.1];
    
%% Q3 ���ſɱȾ�����ؽڿռ�켣
% ��ȡ�켣��Ϣ
   load a1.mat                                          % �켣���ݶ�Ӧ����Ϊsaveddata��������������x,y�ȣ��켣Ϊ��д��ĸa�Ĺ켣
   trajactory_length = size(saveddata.x,2);             % ��ȡ�켣����
   trajcoord = [saveddata.x',saveddata.y'];             % ��ȡ�켣����
   trajspeed = [saveddata.vx',saveddata.vy'];           % ��ȡ�켣�ٶ� 
   dt = saveddata.times(2) - saveddata.times(1);        % ��ȡʱ����
   trajcoord(:,1) = trajcoord(:,1) - 1;                 % �ı�һ�¹켣��λ�ã������е���˶�

%% ���ſɱȾ�����⣨�����������˶�ѧ���е�۳�ʼ��̬��<----------------
        
    thetaB  = zeros(trajactory_length,2);                    % ��ʼ����е�۹ؽڽ�
    midtrajB = zeros(trajactory_length,2);                   % ��ʼ���ؽ�1��λ��
%     thetaB_dot=zeros(trajactory_length,2);                    % ��ʼ����е�۹ؽڽ��ٶ�
  
    
    
%% ��ͼ
% ���ɶ���
    figure
    Robotarm = VideoWriter('Robotarm2.avi');                     % �½���Robotarm.avi���ļ�
    open(Robotarm);                                             % ��Robotarm.avi���ļ�

    axis([-2 1.3 -1.8 1.5])                                     % �̶�������
    hold on
    plot(trajcoord(:,1),trajcoord(:,2),'r-','linewidth',2);     % �����켣    

    M=moviein(trajactory_length);                               % ǰ��Ҫ��plot����moviein��ʼ��
    
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
        h1 = line([0 midtrajB(k,1)],[0 midtrajB(k,2)],'LineWidth',3);                             % ����1
        h2 = line([midtrajB(k,1) trajcoord(k,1)],[midtrajB(k,2) trajcoord(k,2)],'LineWidth',3);   % ����2
        h3 = plot(midtrajB(k,1),midtrajB(k,2),'bo','LineWidth',6);                                % ���ؽ�1

        M(:,k)=getframe;                                        % ץȡͼ����Ϊ��Ӱ�Ļ���
        writeVideo(Robotarm,M(:,k));
        delete(h1);
        delete(h2);
        delete(h3);
    end
    
%     movie(M,1,50);                                              % ��ÿ��30֡���ٶȲ���1��
    close(Robotarm);                                            % �ر�
    
 