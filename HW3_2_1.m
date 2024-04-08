% HW3 Attitude Control with Reaction Wheel
% 2.1 Small-angle attitude maneuver
clc;clear;

%% Parameter
J=[90.5 23.1 -12.3;
   23.1 154.2 18.1;     % MOI of satellite (kg*m^2)
  -12.3 18.1 177.7];

W=1/sqrt(3)*[-1 -1 1 1; -1 1 1 -1; 1 1 1 1;]; % Mounting matrix


%% Case 1 
q_eci_b=[0; 0; 0; 1]; % initial attitude

axis=[1; 0; 0]; % rotaion axis
angle_com=deg2rad(10); % rotation angle (rad)
q_eci_c=[axis*sin(angle_com/2); cos(angle_com/2)]; %Attitude command quaternion

q_e=quatproduct(q_eci_c,quatinv(q_eci_b)); %initial error quaternion


w=[0; 0; 0]; %initial angular rate
trw=[0; 0; 0; 0;]; %initial torque of reaction wheel
h=[0; 0; 0; 0]; %initial angular momentum
Kp=30; %P gain
Kd=-10; %D gain

x=[q_eci_b; w; trw;]; %initial state
x_history=[];

dt=0.01; %sec
t=0; %sec

while abs(2*acos(q_eci_c(4))-2*acos(x(4)))>0.001
    x=RK4(@eq_xdot,t,x,Kd,Kp,J,W,q_eci_c,dt);
    x_history=[x_history, x];
    t=t+dt;
end

maneuver_time=t;


%% function
function q_out=quatproduct(p,q)
p1=p(1); p2=p(2); p3=p(3); p4=p(4);
q_out=[p4 p3 -p2 p1;
      -p3 p4  p1 p2;
       p2 -p1 p4 p3;
      -p1 -p2 -p3 p4]*q;
end

function q_out=quatinv(q)
q(1)=-q(1);
q(2)=-q(2);
q(3)=-q(3);
q_out=q;
end

function qdot=q_derivative(w,q)
wx=w(1); wy=w(2); wz=w(3);
qdot=1/2*[0 wz -wy wx; -wz 0 wx wy; wy -wx 0 wz; -wx -wy -wz 0]*q;
end

function xdot=eq_xdot(t,x,Kd,Kp,J,W,q_eci_c)
q=x(1:4);
q_e=quatproduct(q_eci_c,quatinv(q));
w=x(5:7);
h=x(8:11);
Trw=[2*Kp*q_e(1)*q_e(4)+Kd*w(1);
     2*Kp*q_e(2)*q_e(4)+Kd*w(2);
     2*Kp*q_e(3)*q_e(4)+Kd*w(3)];
trw=pinv(W)*Trw;
hdot=-trw;
wdot=J\(Trw-cross(w,W*h+J*w));
qdot=q_derivative(w,q);
xdot=[qdot; wdot; hdot];
end

function [x]=RK4(Func,t,x,Kd,Kp,J,W,q_eci_c,dt)
k1=Func(t,x,Kd,Kp,J,W,q_eci_c);
k2=Func(t+0.5*dt,x+k1*0.5*dt,Kd,Kp,J,W,q_eci_c);
k3=Func(t+0.5*dt,x+k2*0.5*dt,Kd,Kp,J,W,q_eci_c);
k4=Func(t+dt,x+k3*dt,Kd,Kp,J,W,q_eci_c);
x=x+dt/6*(k1+2*k2+2*k3+k4);
end