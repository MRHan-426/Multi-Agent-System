% Dynamic modeling
syms l1 l2 m1 m2 g;
syms q1 q2 dq1 dq2 ddq1 ddq2;

%% Parameter initialization
R{1}=[cos(q1) -sin(q1) 0;sin(q1) cos(q1) 0;0 0 1];
R{2}=[cos(q2) -sin(q2) 0;sin(q2) cos(q2) 0;0 0 1];
R{3}=[1 0 0;0 1 0;0 0 1];
% Coordinate system origin displacement, use P{1} to represent the relationship between the origin of coordinate system 1 and the origin of coordinate system 0, and use P{2} to represent the relationship between the origin of coordinate system 2 and the origin of coordinate system 1.
P = cell(1,3);
P{1}=[0;0;0];P{2}=[l1;0;0];P{3}=[l2;0;0];
% Position vector of each link's center of mass
Pc = cell(1,3);
Pc{1}=[0;0;0];Pc{2}=[l1;0;0];Pc{3}=[l2;0;0];
% Link mass
m = cell(1,3);
m{2}=m1;
m{3}=m2;
% Moment of inertia tensor
I = cell(1,3);
I{2}=[0;0;0];
I{3}=[0;0;0];

% Angular velocity and angular acceleration between links
w = cell(1,3);dw = cell(1,3);
w{1}=[0;0;0];dw{1}=[0;0;0];% The robot base does not rotate
% Acceleration of the origin and center of mass of each link
dv = cell(1,3);dvc = cell(1,3);
dv{1}=[0;g;0];% Gravity factor

% Joint velocity and acceleration
dq = cell(1,3); ddq = cell(1,3);
dq{2}=[0;0;dq1];dq{3}=[0;0;dq2];
ddq{2}=[0;0;ddq1];ddq{3}=[0;0;ddq2];

% End effector has no force
f = cell(1,4);n = cell(1,4);
f{4}=[0;0;0];
n{4}=[0;0;0];

%% Build kinematic equations
% Forward iteration
for i=1:2 % Matlab indexing starts from 1
    w{i+1}=R{i}.'*w{i}+dq{i+1};
    dw{i+1}=R{i}.'*dw{i}+cross(R{i}.'*w{i},dq{i+1})+ddq{i+1};
    dv{i+1}=R{i}.'*(cross(dw{i},P{i})+cross(w{i},cross(w{i},P{i}))+dv{i});
    dvc{i+1}=cross(dw{i+1},Pc{i+1})+cross(w{i+1},cross(w{i+1},Pc{i+1}))+dv{i+1};

    F{i+1}=m{i+1}*dvc{i+1};
    N{i+1}=[0;0;0];
end% Assuming the mass is concentrated, the inertia tensor of each link is 0


% Backward iteration
for i=3:-1:2
f{i}=R{i}*f{i+1}+F{i};
n{i}=N{i}+R{i}*n{i+1}+cross(Pc{i},F{i})+cross(P{i},R{i}*f{i+1});
end

% Torque
tau = cell(1,2);
tau{1} = n{2}(3);
tau{2} = n{3}(3);
celldisp(tau)