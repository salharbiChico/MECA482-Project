%% MECA482, 05/18/2020,Group 13 Project, 
% PID Controller Example
% Parameters and attempt one

m_p = 0.5; 
m = 0.5;   
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;
q = (m_p+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');
P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((m_p + m)*m*g*l)*s/q - b*m*g*l/q);

Kp = 50;
Ki = 5;
Kd = 50;
C = pid(Kp,Ki,Kd);
T = feedback(P_pend,C);

t=0:0.01:10;
impulse(T,t)
title({'Response of Pendulum Position to an Impulse Disturbance';'under PID Control: Kp = 50, Ki = 5, Kd = 50'});
% attempt two 
Kp2 = 100;
Ki2 = 1;
Kd2 = 20;
C2 = pid(Kp2,Ki2,Kd2);
T2 = feedback(P_pend,C2);
t2=0:0.01:10;
impulse(T2,t2)
axis([0, 2.5, -0.2, 0.2]);
title({'Response of Pendulum Position to an Impulse Disturbance';'under PID Control Kp2 = 100, Ki2 = 1, Kd2 = 20'});

