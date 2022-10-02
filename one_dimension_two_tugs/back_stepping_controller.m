function [f, integrator] = back_stepping_controller(p, v, pd, vd, dt, M, Ma, D, kp, kd, ki, integrator)
% back stepping controller

% input parameters:
% p, v: current position and velocity
% pd, vd: desired position and its derivate

% output paramters:
% f: controlller force KN
% global integrator;

% c-c force 
u1=v(1);
u2=v(2);
u3=v(3);

MM=M+Ma;
C=[-MM(2,2)*u2*u3-MM(2,3)*u3^2; 
    MM(1,1)*u1*u3; 
    MM(3,2)*u1*u3+(-MM(1,1)+MM(2,2))*u1*u2];
C = diag(C, 0);

lamda = eye(3);
kp = diag(kp, 0);   kd = diag(kd, 0);   ki = diag(ki, 0);

fai = p(3); dot_fai = v(3);
R = [cos(fai), -sin(fai), 0; sin(fai), cos(fai), 0; 0, 0, 1];
Rt = R';
dot_R = dot_fai * [-sin(fai), -cos(fai), 0; cos(fai), -sin(fai), 0; 0, 0, 0];

ep = p - pd;
dot_pr = vd - lamda * ep;
ddot_pr = -lamda * (R * v -vd);
s = R * v - dot_pr;

nu_r = Rt *dot_pr;
dot_nu_r = Rt * ddot_pr - Rt * dot_R * Rt * dot_pr;

imax = [100, 100, 100]; imin = -imax;
% integrator = zeros(3, 1);
for i=1:3
    integrator(i) = integrator(i) + ep(i) * dt;
    if(integrator > imax(i))
        integrator(i) =imax(i);
    end
    if (integrator < imin(i))
        integrator(i) =imin(i);
    end
end

% integrate = zeros(3, 1);

f = MM * dot_nu_r + D * nu_r + C * nu_r - Rt * (kp * ep + kd * s + ki * integrator);

f = f /1e3;
end
     
    
    



