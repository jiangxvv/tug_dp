function [tau_r, f, a] = tug_allocation(tau, f0, a0)

% 单位KN

N = 3;
f_range = [0, 900];
a_range = [-pi, pi];
L = [6, -2.7, 0; 6, 2.7, 0; -11, 0, 0];
% f0 = zeros(3, 1);   a0 = zeros(3, 1);

x0 = [f0; a0; zeros(3, 1)];
lb = [f_range(1)*ones(3, 1); a_range(1)*ones(3,1); -1e10*ones(3,1)];
ub = [f_range(2)*ones(3, 1); a_range(2)*ones(3,1); 1e10*ones(3,1)];

options = optimoptions(@fmincon, 'Algorithm', 'sqp', 'display', 'off');
[x, fval] = fmincon(@(x)object_function(x), x0, [], [], [], [], lb, ub, @(x)nonlinear_constrain(x, tau), options);

f = x(1:3);
a = x(4:6);
s = x(7:9);

T = thrusters_configuration(a, L);
tau_r = T*f;

end




%% thruster configuration
function T = thrusters_configuration(a,L)
% function thruster_configuration is used to obtain the configuration
T = zeros(3, 3);

for i = 1 : 3
    T(1,i) = cos(a(i));
    T(2,i) = sin(a(i));
    T(3,i) = L(i,1)*sin(a(i)) - L(i,2)*cos(a(i));
end

end

%% object function
function J = object_function( x )

% 误差权重
P = 1e1*eye(3);
Q = 1e7 * eye(3);
f = x(1:3);
s = x(7:9);
J = s'*Q*s + f'*P*f;
end

%% nonlinear constrain
function [c, ceq] = nonlinear_constrain(x, tau)

f = x(1:3);
a = x(4:6);
s = x(7:9);

L = [6, -2.7, 0; 6, 2.7, 0; -11, 0, 0];
T = thrusters_configuration(a, L);

ceq(1:3) = tau-T*f-s;
c = [];

end