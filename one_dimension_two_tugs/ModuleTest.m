% class DPTug test
%% controller test
clear all; clc;
link = [47.5, 32.5]';
init_position = [0, 0, 0]';
init_velocity = [0, 0, 0]';
dt = 1;
Tug1 = DPTug('Tug1', link,  init_position, init_velocity, dt)
pd = [1, 0, 0]';    vd = [0, 0, 0]'; ad = [0, 0, 0]';
for i = 1:500
    i
    pd = [0.2*i, 0, 0]';    vd = [0.2, 0, 0]'; ad = [0, 0, 0]';
Tug1 = Tug1.setPd(pd, vd, ad);
% Tug1 = Tug1.Controller();
Tug1 = Tug1.BacksteppingController();
% Tug1 = Tug1.ThrusterAllocation();
Tug1 = Tug1.ShipDynamic();
end

subplot(221)
plot(Tug1.time_history_, Tug1.p_history_(1,:),'r-');
hold on 
plot(Tug1.time_history_, Tug1.pd_history_(1, :), 'k-.');
hold off
legend('p', 'pd');
title('p-pd');

subplot(222)
plot(Tug1.time_history_, Tug1.v_history_(1,:),'r-');
hold on 
plot(Tug1.time_history_, Tug1.vd_history_(1, :), 'k-.');
hold off
legend('v', 'vd');
title('v-vd');

subplot(223)
plot(Tug1.time_history_, Tug1.tau_history_(1,:),'r-');
hold on 
plot(Tug1.time_history_, Tug1.tau_r_history_(1, :), 'k-.');
hold off
legend('tau', 'tau_r');
title('tau-tau_r');

subplot(224)
plot(Tug1.time_history_, Tug1.f_history_(1,:),'r-');
hold on 
plot(Tug1.time_history_, Tug1.f_history_(2,:),'k-.');
hold on 
plot(Tug1.time_history_, Tug1.f_history_(3,:),'m--');
hold off
legend('f1', 'f2', 'f3')

% figure;plot(Tug1.time_history_,Tug1.a_history_)
% legend('a1', 'a2', 'a3')

% Tug1.pd_history_
% Tug1 = Tug1.setTau([100; 0; 0])
% Tug1.tau_

%% Tug_Thrust_Allocation test
f0 = [100; 100; 100];
a0 = [-45; 45; 135];

tau = [100; 0; 0];
[f,df,a,da,tau_r,dtau] = Tug_Thrust_Allocation(f0,a0,tau)