clc;    clear all;
global  integrator;
integrator = zeros(3, 1);
f0 = zeros(3, 1);   a0 = zeros(3, 1);
p0 = zeros(3, 1);   v0 = zeros(3, 1);
pd = [2, 0, 0]';    vd = [0, 0, 0]';   ad = zeros(3, 1);

dt0=0.5;       % time interval of the mothership 
time=10;       % overal time of the simulation
t0_list=0:dt0:time;  % time list of simulation
t0=0;           % current time

pd0_history=zeros(3,length(t0_list));   vd0_history=zeros(3,length(t0_list));
p0_history=zeros(3,length(t0_list));    v0_history=zeros(3,length(t0_list));
F0_history=zeros(3,length(t0_list));    Ft_history=zeros(3,length(t0_list));
for i = 1 : length(t0_list)
    p0_history(:, i) = p0;  v0_history(:, i) = v0;
    [p, v] = DP_tug(p0, v0, pd, vd, ad);
    p0 = p;     v0 = v;
    
end

subplot(121)
plot(t0_list, p0_history);
subplot(122)
plot(t0_list, v0_history);

%%
clear all; clc;
f0 = [100; 100; 100];   a0 = pi/2*[1, 1, 1]';
tau = [2800, 0, 0]';

% [f,df,a,da,tau_r,dtau] = Tug_Thrust_Allocation(f0,a0,tau)

[tau_r, f, a] = tug_allocation(tau, f0, a0)

%% tug_thruster_allocation test
clear all;  clc;
f0 = zeros(3, 1);   a0 = zeros(3, 1);

% tau = [2000, 0, 0]'
% [f,df,a,da,tau_r,dtau] = Tug_Thrust_Allocation(f0,a0,tau);
% tau_r

N = 100;
tau_r_history = zeros(3, N);    tau_history = zeros(3, N);
a_history = tau_history;    f_history = a_history;
tau_history(1,:) = 50:-1:-49;
for i = 1 : N
    tau = tau_history(:, i);
%     [tau_r, f, a] = tug_allocation(tau, f0, a0);
    [f,df,a,da,tau_r,dtau] = Tug_Thrust_Allocation(f0,a0,tau);
    f0 = f; a0 = a;
    f_history(:, i) = f;    a_history(:, i) = a;
    tau_r_history(:, i) = tau_r;
end
subplot(221)
plot(1:N, tau_r_history(1, :));
hold on 
plot(1:N, tau_history(1, :))
hold off
legend('tau_r', 'tau')
subplot(222)
plot(1:N, f_history);
title('f');
subplot(223)
plot(1:N, a_history);
title('a');




%%
clear all; clc;
f0 = zeros(3, 1);   a0 = zeros(3, 1);
tau = [-2000, 0, 0]';
% ���Ƕ�ת��Ϊ����
d2r = pi/180;
% ��һ���ƽ���ת�ǻ�Ϊ����
a0 = a0 * d2r;
% ȫ��ת�ƽ�������
N = 3;
% �ɳ����ӵ�Ȩ����
Q = 1e12*diag([30 30 1]);
% ȫ��ת�ٶȵ�Ȩ����
Omega = 10*eye(N);
% �ƽ���ȫ��ת�ٶ�����(rad/sample_time)
da = [-180 180]*d2r;                                % �谴ʵ������
% �����ƽ��������ı仯����(KN)
Frange = [0 900];                              % �谴ʵ������
% ������ʹ�����
% delta_u = [0; 1; 1; 1; 0; 1; 1; 1;];            % ����������ʧЧ���
% �ƽ��������ı仯�ٶ�����(KN/sample_time)
dF = [-1800 1800];                                  % �谴ʵ������
% �ƽ�������ֵ
% tug(32m*12m)
L = [6, -2.7, 0; 6, 2.7, 0; -11, 0, 0];
% L = [76 0; 51 26; -51 26; -76 0; -51 -26; 51 -26];     % �谴ʵ������
% Semi 708
% L = [15.7 35.5 -21.5; 47.02 24.58 -21.5; 47.02 -24.58 -21.5; 15.7 -35.5 -21.5; ...
%      -15.7 -35.5 -21.5;-47.02 -24.58 -21.5; -47.02 24.58 -21.5; -15.7 35.5 -21.5;];
% Semi 807
% L = [15.7 24.58; 47.02 35.5; 47.02 -35.5; 15.7 -24.58; -15.7 -24.58; -47.02 -35.5; -47.02 35.5; -15.7 24.58;];     % �谴ʵ������ Semi807
% Semi 981
% L = [47.02 35.5; 47.02 24.58; 47.02 -24.58; 47.02 -35.5; -47.02 -35.5; -47.02 -24.58; -47.02 24.58; -47.02 35.5;];     % �谴ʵ������

% Ŀ�꺯������λ�óͷ�����ӳ���
pp = 1000;
% Ŀ�꺯������λ�óͷ����ĸ�еĳ���
ee = 1;

% ��ֹ�����ú�Ŀ������䣬1��4�ƽ���������������
% angle_sector1o = [-15 15;75 285]*d2r;
% angle_sector2o = [165 425]*d2r;
% angle_sector3o = [115 375]*d2r;
% angle_sector4o = [165 195;-105 105]*d2r;
% angle_sector5o = [-15 245]*d2r;
% angle_sector6o = [-65 195]*d2r;
angle_sector1o = [-180 180]*d2r;
angle_sector2o = [-180 180]*d2r;
angle_sector3o = [-180 180]*d2r;
angle_sector4o = [-180 180]*d2r;
angle_sector5o = [-180 180]*d2r;   
angle_sector6o = [-180 180]*d2r;
angle_sector7o = [-180 180]*d2r;   
angle_sector8o = [-180 180]*d2r;
% �����������䣬���Ƕ������ƽ���������ѡ��δ�����ܼ��붯̬��ֹ��
angle_sector = 1000*ones(N,2);
angle_sector(1,:) = angle_sector1o;
angle_sector(2,:) = angle_sector2o;
angle_sector(3,:) = angle_sector3o;
angle_sector(4,:) = angle_sector4o;
angle_sector(5,:) = angle_sector5o;
angle_sector(6,:) = angle_sector6o;
angle_sector(7,:) = angle_sector7o;
angle_sector(8,:) = angle_sector8o;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �ƽ���ȫ��ת�ٶ�Լ������
dda = 1000*ones(N,2);     % ��ʼ��
% ���ȿ��ǿ����������ϲ��Ƕȵ�����,����ddaת��[-2pi 2pi],�Ա���da�Ƚ�
for i = 1 : N
    dda(i,:) = angle_sector(i,:) - a0(i);
    while(dda(i,2) + 100*eps < 0)
        dda(i,:) = dda(i,:) + 2*pi;
    end
    while(dda(i,1) - 100*eps > 0)
        dda(i,:) = dda(i,:) - 2*pi;
    end
end
% ��ȫ��ת�ƽ���ÿ�������ת��da�Ƚϣ�ѡ����С�ı仯����
for i = 1 : N
    if da(1) > dda(i,1)
        dda(i,1) = da(1);
    end
    if da(2) < dda(i,2)
        dda(i,2) = da(2);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u0 = 1;
% �ƽ��������仯����
F = [ Frange(1)*u0 Frange(2)*u0 ]; % ����ʧЧ���������������仯����
ff = 1e10*ones(N,2);
for i = 1 : N
    ff(i,1) = f0(i) + dF(1);
    if ff(i,1) <F(1)
        ff(i,1) = F(1);
    end
    
    ff(i,2) = f0(i) + dF(2);
    if ff(i,2) > F(2);
        ff(i,2) = F(2);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



T =  thrusters_configuration(a0,L);
dTf = get_coefficient(f0, a0, L);
% dTf = Get_Coefficients(f0,a0,pp,ee);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ϊ���ι滮����׼��ϵ��
H = blkdiag(2*eye(N),2*Omega,2*Q);
I = [];
A = [];
b = [];
Aeq = [T,dTf,eye(3)];
beq = tau;

% lb = [ff(:,1);dda(:,1);-1e10*ones(3,1)];
% ub = [ff(:,2);dda(:,2); 1e10*ones(3,1)];

lb = [zeros(3, 1); -pi*ones(3,1); -1e10*ones(3,1)];
ub = [Frange(2)*ones(3, 1); pi*ones(3,1); 1e10*ones(3,1)];
x0 = [f0; zeros(N,1); zeros(3,1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �����ι滮����
options = optimset('Algorithm','interior-point-convex','Display','off');
x = quadprog(H,I,A,b,Aeq,beq,lb,ub,x0,options);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �������
f = x(1:N);
df = f - f0;
da = x(N+1:2*N);
a = a0 + da;
da = da / d2r;
a = a / d2r;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ��aת��[-pi pi]���䣬�Է��㻭ͼ
for i = 1 : N
    while a(i) > 200
        a(i) = a(i) - 360;
    end
    while a(i) <-200
        a(i) = a(i) + 360;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ʵ������
T =  thrusters_configuration(a*d2r, L);
% T1 = thrusters_configuration6(a*d2r);
tau_r = T*f;
% tau_r6 = T1*f;
dtau = tau_r - tau;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%% leader_reference_mode test
% function pd = leader_reference_mode(t, dt, w, z, r)

m0 = diag([5.25e7, 5.25e7, 7.5e10]);
ma0 = diag([1.7e7, 4.9e7, 5.4e10]);
D0 = diag([2.4e6, 4.8e6, 3.9e7]);
M0 = m0 + ma0;

dt = 0.5;
w = 0.005;  z = 0.9;
r = 100;
delta = 0;

y2 = zeros(
for i=1:N+1,
   y2(i,:) = [x v];   
   x_dot = v;
   v_dot = w^2*(r2-x) - 2*z*w*v - delta*abs(v)*v;
   v = v + dt*v_dot;
   x = x + dt*x_dot;
end

% end


%% 
% ExRefMod   2nd-order reference model with nonlinear damping and velocity saturation
% Author:    Thor I. Fossen
% Date:      3rd November 2001
% Revisions: 

z = 1;     % relative damping ratio
w = 1;     % natural frequency
delta = 1; % nonlinear damping coeff.
vmax = 1;  % max velocity
h = 0.1;   % sampling time
N = 200;   % number of samples

t  = 0:h:h*N;

% linear mass-damper-spring system
r1 = 10*ones(max(size(t)),1);
[A,B,C,D] = ord2(w,z); 
[x1,y1]   = lsim(A,B,C,D,r1,t); 

r2 = 10*r1;
[A,B,C,D] = ord2(w,z); 
[x2,y2]   = lsim(A,B,C,D,r2,t); 

% nonlinear damping
x = 0;
v = 0;
r2 = 10;
y2 = zeros(N+1,2);
for i=1:N+1,
   y2(i,:) = [x v];   
   x_dot = v;
   v_dot = w^2*(r2-x) - 2*z*w*v - delta*abs(v)*v;
   v = v + h*v_dot;
   x = x + h*x_dot;
end

% velocity saturation
x = 0;
v = 0;
r3 = 10;
y3 = zeros(N+1,2);
for i=1:N+1,
   y3(i,:) = [x v];   
   v_dot = w^2*(r3-x) - 2*z*w*v;
   x_dot = v;
   v = v + h*v_dot;
   if abs(v)>vmax,       % saturation
      v = sign(v)*vmax;
   end
   x = x + h*x_dot;
end

% plots
figure(gcf)
subplot(211); plot(t,y1(:,1),'linewidth',2)
hold on; plot(t,y2(:,1),'--k',t,y3(:,1),'r'); hold off; grid
title('2nd-order mass-damper-spring reference model')
legend('linear damping','nonlinear damping','velocity saturation')
subplot(212); plot(t,y1(:,2),'linewidth',2);
hold on; plot(t,y2(:,2),'--k',t,y3(:,2),'r'); hold off; grid



    