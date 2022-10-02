function [f, df, a, da, tau_r, dtau] = thruster_allocate_quadprog(f0, a0, tau)
% INPUT:
%  f0               上一步的推力值向量 (N)
%  a0               上一步的推进器转角向量
%  tau              本步所要求的推力向量 (N;N;N*m)
%
% OUTPUT:
%  f                分配后的各推进器推力值向量 (N)
%  df               相对于上一步的推力值变化 (N)
%  a                分配后的各推进器转角向量 
%  da               相对于上一步的转角变化 
%  tau_r            分配后实际推力向量 (N;N;N*m)
%  dtau             实际推力与要求推力的差值
%
%%%%%%%%%%%%%%%%%%%%%%%%%% 常量 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%拖船个数
N=4;

%角度转成弧度
d2r = pi/180;

%性能指标
P = eye(N);                  % 能量消耗权矩阵
Q = 1e12*diag([30 30 1]);  % 松弛因子的权矩阵
Omega = 10*eye(N);         % 全回转速度的权矩阵

% 缆绳连接位置
L=[47.5, 47.5, -47.5, -47.5; ...
    32.5, -32.5, -32.5, 32.5];

% 角度范围和角度变化范围
a_max=[pi/2; pi/2; 3/2*pi; 3/2*pi];
a_min=[-pi/2; -pi/2; pi/2; pi/2];
da_max=10/180*pi; %最大转角增量，暂定为10度

% 力的范围和力的变化率的范围
f_max=1e11*[1; 1; 1; 10]; %N
f_min=zeros(N, 1);
f_ratio=1; %力增量变化率，定为原力的100%

% 推力和角度的上下限， ub<f, da, s<ub
lb=zeros(11,1);
ub=zeros(11,1);

ub(1:N)=f_max;
lb(1:N)=f_min;
ub(N+1:2*N)=da_max;
lb(N+1:2*N)=-da_max;
ub(9:11)=0.01*tau;
lb(9:11)=-0.01*tau;
ub;
lb;
for i=1:N
    if (a0(i)+da_max>a_max(i))
        ub(i+4)=a_max(i);
    else
        ub(i+4)=a0(i)+da_max;
    end
    
    if (a0(i)-da_max>a_min(i))
        lb(i+4)=a0(i)-da_max;
    else
        lb(i+4)=a_min(i);
    end
    
%     if ((1+f_ratio)*f0(i)>f_max(i))
%         ub(i)=f_max(i);
%     else
%         ub(i)=(1+f_ratio)*f0(i);
%     end
%     
%     if ((1-f_ratio)*f0(i)>f_min(i))
%         lb(i)=(1-f_ratio)*f0(i);
%     else
%         lb(i)=f_min(i);
%     end
end


dTf=get_coefficient(f0, a0, L);
B0=thrusters_configuration(a0, L);


H = blkdiag(2*eye(N),2*Omega,2*Q);
I = [];
A = [];
b = [];
Aeq = [B0, dTf, eye(3)];
beq = tau;
x0 = [f0; zeros(N,1); zeros(3,1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 求解二次规划问题
x=zeros(2*N+3);
options = optimset('Algorithm','interior-point-convex','Display','off');
x = quadprog(H,I,A,b,Aeq,beq,lb,ub,x0,options);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 函数输出
f = x(1:N);
df = f - f0;
da = x(N+1:2*N);
a = a0 + da;
% da = da / d2r;
% a = a / d2r;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 实际推力
B =  thrusters_configuration(a, L);
% T1 = thrusters_configuration6(a*d2r);
tau_r = B*f;
% tau_r6 = T1*f;
dtau = tau_r - tau;

end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%function to get deriviate
function dTf = get_coefficient(f0, a0, L)
%
% f1 = f0(1); f2 = f0(2); f3 = f0(3); f4 = f0(4); 
% 
% a1 = a0(1); a2 = a0(2); a3 = a0(3); a4 = a0(4);
% %
% dTf = [                        -f1*sin(a1),                                -f2*sin(a2),                                -f3*sin(a3),                            -f4*sin(a4);
%                             f1*cos(a1),                                 f2*cos(a2),                                 f3*cos(a3),                             f4*cos(a4);
% f1*((157*cos(a1))/10 + (71*sin(a1))/2), f2*((2351*cos(a2))/50 + (1229*sin(a2))/50), f3*((2351*cos(a3))/50 - (1229*sin(a3))/50), f4*((157*cos(a4))/10 - (71*sin(a4))/2)];

dTf=zeros(3, 4);
for i = 1:4
    dTf(1, i) = -f0(i)*sin(a0(i));
    dTf(2, i) = f0(i)*cos(a0(i));
    dTf(3, i) = f0(i)*(L(2, i)*sin(a0(i))+L(1, i)*cos(a0(i)));
end

end


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function to get the configuration matrix B(a)
function B = thrusters_configuration(a, L)

B = zeros(3, 4);

for i = 1:4
    B(1, i) = cos(a(i));
    B(2, i) = sin(a(i));
    B(3, i) = -L(2, i)*cos(a(i))+L(1, i)*sin(a(1));
end

end





