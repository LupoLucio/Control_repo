clear all
close all
clc

%Definizione specifiche
Tr = 0.25;
Mp = 0.2;

%Calcolo sample time
omega_n = 1.8/Tr;
Ts_max = (2*pi)/(10*omega_n);
Ts = Ts_max / 4;

%Definizione P(s)
s = tf('s');
P_c = 90 / (s^2 + 60*s + 3);

%Calcolo P(z)
P_d = c2d(P_c, Ts, 'zoh');

%State-space discrete model
ssP_d = ss(P_d);

%Calcolo matrici discrete
[Phi, Gamma, H, J] = ssdata(ssP_d);

%Calcolo di Nx e Nu
n = size(Phi, 1);       %ordine del sistema
p = size(H, 1);         %numero di uscite
m = size(Gamma, 2);     %numero di ingressi

M = [Phi - eye(n), Gamma; H, zeros(p, m)];
ris = [zeros(n, 1); 1];
sol = inv(M) * ris;
Nx = sol(1 : n);
Nu = sol(n+1 : end);

%Piazzamento dei poli
delta = log(1/Mp) / (sqrt(pi^2 + log(1/Mp)^2));

p_c_1 = -omega_n*delta + 1i*omega_n*sqrt(1-delta^2);        %poli nel continuo
p_c_2 = conj(p_c_1);

p_d = [exp(p_c_1*Ts), exp(p_c_2*Ts)];                       %poli nel discreto

%Calcolo di K
K = place(Phi, Gamma, p_d);

%Full-order state observer
poles_obs_d = [exp(-40*Ts), exp(-50*Ts)];         %Piazzamento poli observer molto prima dei poli della catena chiusa

L = (place(Phi', H', poles_obs_d))';              %Gain observer

Phi_obs = Phi - L*H;                              %Matrici discrete observer
Gamma_obs = [Gamma, L];
H_obs = eye(n);
J_obs = zeros(n, m+p);
