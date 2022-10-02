function [p, v]=ship_dynamic( dt, p0, v0, M, Ma, D, F)
% solve the ship dynamic equations with the R-k4 method
% integral one step 

% input parameters:
% t: current time
% dt: time interval(time step)
% p0: current position of vessel
% vo: current velocity of vessel
% M, Ma,D: parameters of vessel
% F: actuation 

% output parameters:
% p: position of next step
% v: velocity of next step

% yaw angle
fai = p0(3);

% rotation matrix
R=[cos(fai), -sin(fai) 0; 
   sin(fai), cos(fai), 0;
   0,         0,       1];

% c-c force 
u1=v0(1);
u2=v0(2);
u3=v0(3);

MM=M+Ma;
C=[-MM(2,2)*u2*u3-MM(2,3)*u3^2; 
    MM(1,1)*u1*u3; 
    MM(3,2)*u1*u3+(-MM(1,1)+MM(2,2))*u1*u2];

% R-k4 method
k1=zeros(6,1);
k2=zeros(6,1);
k3=zeros(6,1);
k4=zeros(6,1);

% state, position and velocity
y=zeros(6,1);
y=[v0; p0];

v0_temp=zeros(3,1);

% k1
k1(1:3)=inv(MM)*(F-C-D*v0);

k1(4:6)=R*v0;
% k1(4:6)=R*k1(1:3);





% k2
y_temp=y+dt/2*k1;%вт╠Да©

v0_temp=y_temp(1:3);
u1=v0_temp(1);
u2=v0_temp(2);
u3=v0_temp(3);
C=[-MM(2,2)*u2*u3-MM(2,3)*u3^2; 
    MM(1,1)*u1*u3; 
    MM(3,2)*u1*u3+(-MM(1,1)+MM(2,2))*u1*u2];
fai=y_temp(6);
R=[cos(fai), -sin(fai) 0; 
   sin(fai), cos(fai), 0;
   0,         0,       1];

k2(1:3)=inv(MM)*(F-C-D*v0_temp);
k2(4:6)=R*y_temp(1:3);



% k3
y_temp=y+dt/2*k2;

v0_temp=y_temp(1:3);
u1=v0_temp(1);
u2=v0_temp(2);
u3=v0_temp(3);
C=[-MM(2,2)*u2*u3-MM(2,3)*u3^2; 
    MM(1,1)*u1*u3; 
    MM(3,2)*u1*u3+(-MM(1,1)+MM(2,2))*u1*u2];
% update the rotation matrix
fai=y_temp(6);
R=[cos(fai), -sin(fai) 0; 
   sin(fai), cos(fai), 0;
   0,         0,       1];

k3(1:3)=inv(MM)*(F-C-D*v0_temp);
k3(4:6)=R*y_temp(1:3);



% k4
y_temp=y+dt*k3;

v0_temp=y_temp(1:3);
u1=v0_temp(1);
u2=v0_temp(2);
u3=v0_temp(3);
C=[-MM(2,2)*u2*u3-MM(2,3)*u3^2; 
    MM(1,1)*u1*u3; 
    MM(3,2)*u1*u3+(-MM(1,1)+MM(2,2))*u1*u2];
% update the rotation matrix
fai=y_temp(6);
R=[cos(fai), -sin(fai) 0; 
   sin(fai), cos(fai), 0;
   0,         0,       1];

k4(1:3)=inv(MM)*(F-C-D*v0_temp);
k4(4:6)=R*y_temp(1:3);
% k4(4:6)=R*k4(1:3);

% k4(4:6)=R*y_temp(4:6);
% k4(1:3)=R*v0;
% k4(4:6)=inv(MM)*(F-C-D*v0_temp);

% sum k1~k4
y_next=y+dt/6*(k1+2*k2+2*k3+k4);

% position and velocity of next step
p=y_next(4:6);
v=y_next(1:3);




end


function R=Rotation_Matrix(p)
fai=p(3);
R=[cos(fai), -sin(fai) 0; 
   sin(fai), cos(fai), 0;
   0,         0,       1];
end