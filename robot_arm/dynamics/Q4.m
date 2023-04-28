% 动力学建模
syms l1 l2 m1 m2 g;
syms q1 q2 dq1 dq2 ddq1 ddq2;

%% 参数初始化
R{1}=[cos(q1) -sin(q1) 0;sin(q1) cos(q1) 0;0 0 1];
R{2}=[cos(q2) -sin(q2) 0;sin(q2) cos(q2) 0;0 0 1];
R{3}=[1 0 0;0 1 0;0 0 1];
% 坐标系原点位移，用P{1}表示坐标系1与坐标系0原点位置关系，用P{2}表示坐标系2与坐标系1原点位置关系。
P = cell(1,3);
P{1}=[0;0;0];P{2}=[l1;0;0];P{3}=[l2;0;0];
% 每个连杆质心的位置矢量
Pc = cell(1,3);
Pc{1}=[0;0;0];Pc{2}=[l1;0;0];Pc{3}=[l2;0;0];
% 连杆质量
m = cell(1,3);
m{2}=m1;
m{3}=m2;
% 惯性张量
I = cell(1,3);
I{2}=[0;0;0];
I{3}=[0;0;0];

% 连杆间角速度和角加速度
w = cell(1,3);dw = cell(1,3);
w{1}=[0;0;0];dw{1}=[0;0;0];% 机器人底座不旋转
% 连杆坐标原点和质心加速度
dv = cell(1,3);dvc = cell(1,3);
dv{1}=[0;g;0];% 重力因素

% 关节速度和加速度
dq = cell(1,3); ddq = cell(1,3); 
dq{2}=[0;0;dq1];dq{3}=[0;0;dq2];
ddq{2}=[0;0;ddq1];ddq{3}=[0;0;ddq2];

% 末端执行器没有力
f = cell(1,4);n = cell(1,4);
f{4}=[0;0;0];
n{4}=[0;0;0];

%% 建立运动学方程
% 外推
for i=1:2 %matlab下标从1开始
w{i+1}=R{i}.'*w{i}+dq{i+1};
dw{i+1}=R{i}.'*dw{i}+cross(R{i}.'*w{i},dq{i+1})+ddq{i+1};

dv{i+1}=R{i}.'*(cross(dw{i},P{i})+cross(w{i},cross(w{i},P{i}))+dv{i});
dvc{i+1}=cross(dw{i+1},Pc{i+1})+cross(w{i+1},cross(w{i+1},Pc{i+1}))+dv{i+1};

F{i+1}=m{i+1}*dvc{i+1};
N{i+1}=[0;0;0];%假设质量集中，每个连杆惯性张量为0
end

% 内推
for i=3:-1:2
    f{i}=R{i}*f{i+1}+F{i};
    n{i}=N{i}+R{i}*n{i+1}+cross(Pc{i},F{i})+cross(P{i},R{i}*f{i+1});
end

% 力矩
tau = cell(1,2);
tau{1} = n{2}(3);
tau{2} = n{3}(3);
celldisp(tau)