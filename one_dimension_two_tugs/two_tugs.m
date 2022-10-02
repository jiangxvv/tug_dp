clear all; clc;
%% 参数
% DPreal, qt版本的DP程序
m0 = diag([5.25e7, 5.25e7, 7.5e10]);
ma0 = diag([1.7e7, 4.9e7, 5.4e10]);
D0 = diag([2.4e6, 4.8e6, 3.9e7]);
M0 = m0 + ma0;


% parameters of the tug1
M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
Ma1=[7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
D1=[(M1(1,1)+Ma1(1,1))/100, 0, 0; 0, (M1(2,2)+Ma1(2,2))/40, 0; 0, 0, (M1(3,3)+Ma1(3,3))/20];

global f0 a0 integrator;

%% 初始化参数
p0 = zeros(3, 1);   v0 = zeros(3, 1);
p1 = [448; 0; 0];   v1 = zeros(3, 1);
p2 = [-448; 0; 0];  v2 = zeros(3, 1);

% guidance for the mothership (trajectory, open loop), time
dt0=0.5;       % time interval of the mothership 
time=200;       % overal time of the simulation
t0_list=0:dt0:time;  % time list of simulation
t0=0;           % current time

pd0_history=zeros(3,length(t0_list));   vd0_history=zeros(3,length(t0_list));
p0_history=zeros(3,length(t0_list));    v0_history=zeros(3,length(t0_list));
F0_history=zeros(3,length(t0_list));    Ft_history=zeros(3,length(t0_list));

pd1_history=zeros(3,length(t0_list));   vd1_history=zeros(3,length(t0_list));
p1_history=zeros(3,length(t0_list)*(num+1));    v1_history=p1_history;
F1_history=zeros(3,length(t0_list)*(num+1));    pd_t1_history=p1_history;

pd2_history=zeros(3,length(t0_list));   vd2_history=zeros(3,length(t0_list));
p2_history=zeros(3,length(t0_list)*(num+1));    v2_history=p1_history;
F2_history=zeros(3,length(t0_list)*(num+1));    pd_t2_history=p1_history;


%% 主程序（循环）
for i = 1:length(t0_list)
    t0 = t0_list(i);
    
    p0_history(:, i) = p0;  v0_history(:, i) = v0;
    
    
    
end

