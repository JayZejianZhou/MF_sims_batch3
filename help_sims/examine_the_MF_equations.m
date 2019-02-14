% Zejian Zhou
% 2019-2-13
% this simulation is to prove that the MF equations system works
clc
global A B R Q aNum Pi
%设置系统方程参数
A=[zeros(2),eye(2);zeros(2),zeros(2)];
B=[zeros(2),zeros(2);zeros(2),eye(2)];
R=0.1*eye(4);G=0.1*eye(4);rho=eye(4);
Q=40*eye(4);
%agent number
aNum=100;
%simulation duration and time stamp
T=5;
dt=0.01;
%set initial condition, -20,20 for position
init_state_ref=[100,10,11,0,0,0]';
init_s1=(init_state_ref(1)+1.8-(init_state_ref(1)-1.8)).*rand(1,aNum) + init_state_ref(1)-1.8;
init_s2=(init_state_ref(2)+1.8-(init_state_ref(2)-1.8)).*rand(1,aNum) + init_state_ref(2)-1.8;
%set initial condition, -5,5 for velocity
init_s3= -0 + (-0+0)*rand(1,aNum);
init_s4= -0 + (-0+0)*rand(1,aNum);
save 'data\init_pos.mat' 'init_s1' 'init_s2' 'init_s3' 'init_s4'
load 'data\init_pos.mat'

init_state=zeros(4*aNum,1);
%矩阵交叉合并 state is [x y v1 v2 x y v1 v2 ... x v1 y v2]
for i=1:aNum
    init_state(i*4-3)=init_s1(i);
    init_state(i*4-2)=init_s2(i);
    init_state(i*4-1)=init_s3(i);
    init_state(i*4-0)=init_s4(i);
end

%for ode visibility
options = odeset('OutputFcn',@odeplot);
%%
%计算ARE
Pi=care(A,B,Q);
%仿真state trajectory
obj_bar = sde(@ode_dynamics,@sde_diffusion,'StartState',init_state);
[X1,~] = obj_bar.simulate(T/dt, 'DeltaTime', dt);
%%
function x_out=ode_dynamics(t,x_in)
global Pi aNum A B R Q
x_out=zeros(aNum*4,1);
%disassemble inputs
all_pos=zeros(aNum,2);
all_vel=zeros(aNum,2);
for i=1:aNum
    all_pos(i,:)=[x_in(i*4-3),x_in(i*4-2)];
    all_vel(i,:)=[x_in(i*4-1),x_in(i*4-0)];
end
%take step for each agent
for i=1:aNum
    x=[all_pos(i,:)';all_vel(i,:)'];
    u_drive=-inv(R)*B'*(Pi*(x-h_v(t)));
    u=u_drive;
    xdot=A*(x-h_v(t))+B*u+h_v_dot(t);
    x_out(i*4-3)=xdot(1);
    x_out(i*4-2)=xdot(2);
    x_out(i*4-1)=xdot(3);
    x_out(i*4-0)=xdot(4);
end
end

%diffusion rate
function out=sde_diffusion(t,x)
global aNum
out=2*ones(4*aNum,1);
% out=2*randn(6*aNum,1);
end

% 最终目标
function p=h_v(t)
target=[10,20];
y1=normrnd(target(1),0);
y2=normrnd(target(2),0);
p=[y1;y2;0;0];
end
function p=h_v_dot(t)
p=[0;0;0;0];
end