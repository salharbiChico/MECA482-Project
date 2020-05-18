%% MECA 482, 05/18/2020, Group Ptoject, (Group ID# 13)
clear;close all;clc;
% System parameters
g=9.8; % gravity  [m/s^2]
ro=0.04; % Pulley1 radius [m] 
lo=0.1; % Arm length [m] 
l_rod=0.120; % Pulley1 rod [m] 
l_p=0.30; % Pendulum length [m]  
mo=0.05; % Pulley1 + arm mass [kg]  
m_rod=0.015; % Pulley1 rod mass [m] 
me=0.02; % Encoder mass [kg]
m_p=0.05; %Pendulum mass [m] 
R=12.50; % Motor Resistance  [Ohm] 
K=0.2751; % Motor Constant

% A matrix
A32 = 3*g*lo*m_p/(4*l_rod^2*m_rod+3*lo^2*(m_p+4*me)+6*mo*ro^2);
A33 = -12*K^2/(R*(4*l_rod^2*m_rod+3*lo^2*(m_p+4*me)+6*mo*ro^2));
A42 = 3*(g+9*g*lo^2*m_p/(4*l_rod^2*m_rod+3*lo^2*(m_p+4*me)+6*mo*ro^2))/(2*l_p);
A43 = -18*K^2*lo/(R*l_p*(4*l_rod^2*m_rod+3*lo^2*(m_p+4*me)+6*mo*ro^2));
A = [0 0 1 0; 0 0 0 1; 0 A32 A33 0; 0 A42 A43 0];

% B matrix
B3 = 12*K/(R*(4*l_rod^2*m_rod+3*lo^2*(m_p+4*me)+6*mo*ro^2));
B4 = 18*K*lo/(R*l_p*(4*l_rod^2*m_rod+3*lo^2*(m_p+4*me)+6*mo*ro^2));
B = [0; 0; B3; B4];

% C matrix
C = eye(4);

% D matrix
D = 0;

% LQR
Q = diag([1.5 6 0 0]);
R = 0.0028;
[K, S, EIG] = lqr(A, B, Q, R);
display(K);
display(EIG);

% system simulation
sys = ss(A,B,C,D);
sys_feedback = feedback(sys,K);
step(sys_feedback);