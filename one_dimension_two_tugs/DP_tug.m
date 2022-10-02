function [p, v] = DP_tug(p0, v0, pd, vd, ad)
% parameters of the tug1
global  integrator;
m = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
ma = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
M = m + ma;
D = [M(1,1)/100, 0, 0; 0, M(2,2)/40, 0; 0, 0, M(3,3)/20];

dt = 0.5;
kp = [1e6; 1e6; 1e9];
kd = [1e6; 1e6; 1e9];
ki = [1e2; 1e2; 1e5];
tau = back_stepping_controller(p0, v0, pd, vd, dt, m, ma, D, kp, kd, ki)
% [p, v]=ship_dynamic(dt, p0, v0, m, ma, D, tau*1e3);

f0 = zeros(3, 1);   a0 = zeros(3, 1);   tau_r = zeros(3,1);
fmax = 900;
tau_r(1) = tau(1);
if abs(tau(1))> fmax*3
    tau_r(1) = fmax*3*sign(tau(1));
end
% 
% % [f,df,a,da,tau_r,dtau] = Tug_Thrust_Allocation(f0,a0,tau/1e3);
% % 
tau_r
[p, v]=ship_dynamic( dt, p0, v0, m, ma, D, tau_r*1e3);
end