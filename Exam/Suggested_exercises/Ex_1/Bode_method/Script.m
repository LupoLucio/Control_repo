clear all;
close all;
clc;


% specifiche
Tr = 0.15;
Mp = 0.2;

% matrici
A = [0 1; 0 -10];
B = [0; 120];
C = [1 0];
D = 0;

% state space 
ss_P = ss(A,B,C,D);


poles = pole(ss_P);
zero_poles = sum(abs(poles) < 1e-6);

required_integrators = 2; %Tracking rampa senza errore
controller_integrators = required_integrators - zero_poles;

% costruzione controllore

omega_n = 1.8/Tr;


[Mag_P, Phase_P_deg]= bode(ss_P,omega_n);
Phase_P_rad = Phase_P_deg*pi/180;

Delta_K = 1/Mag_P;
delta = (log(1/Mp))/(sqrt(pi^2+(log(1/Mp))^2));
phi_m_rad = atan((2*delta)/(sqrt(sqrt(1+4*delta^4)-2*delta^2))); %Margine di fase desiderato
Delta_phi_rad = -pi + phi_m_rad - Phase_P_rad;  %Quanta fase manca al processo per raggiungere il margine di fase desiderato

Kp = Delta_K*cos(Delta_phi_rad);
alpha = 100;
T_D = (tan(Delta_phi_rad)+sqrt((tan(Delta_phi_rad))^2+(4/alpha)))/(2*omega_n);
T_I = alpha * T_D;

s = tf('s');

Tl = 1/(4 * omega_n);
H = s/(1+Tl*s); %Derivativo reale
Contr = Kp*(1+(1/(T_I*s))+T_D*H);
P = tf(ss_P);  % tf del processo
L = Contr * P;
T = feedback(L, 1);
info_step = stepinfo(T)