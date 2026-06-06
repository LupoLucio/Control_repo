clear all
close all
clc

Ts = 0.01e-3;

% Dynamic model
A = [0, 1; -2, -40];
B = [0; 40];
C = [1, 0];
D = 0;

P_ss_c = ss(A,B,C,D);

P_ss_d = c2d(P_ss_c,Ts,'zoh');

[Phi, Gamma, H, J] = ssdata(P_ss_d);

n = size(Phi, 1);   % ordine sistema
p = size(H, 1);   % numero uscite
m = size(Gamma, 2);   % numero ingressi

Phi_z = [1, H; zeros(n,1), Phi];
Gamma_z = [zeros(p,m);Gamma];
H_z = [zeros(p,p),H];
J_z = J;


%% Optimal Feedback Controller
% Cost matrices from Bryson's rule
Q = diag(ones(1, n+1) / 10);   % (n+1) x (n+1): n stati + 1 integratore
R = 1/20;


% Feedback gain and resulting closed loop poles
[Kz,~,poles_T_d] = dlqr(Phi_z,Gamma_z,Q,R);
Ki = Kz(1);
K  = Kz(2:end);

% Nx, Nu
M = [Phi - eye(n), Gamma; H, J];

ris = [zeros(n, 1); 1];

sol = inv(M) * ris;

Nx = sol(1:n);
Nu = sol(n+1:end);


%% Observer
% eigenvalues in z domain, 4-5 times faster than CL discrete poles
poles_d_obs = [0.2,0.25];

% Observer Gain
L = place(Phi',H',poles_d_obs)';

Phi_obs = Phi-L*H;
Gamma_obs = [Gamma, L];
H_obs = eye(n);
J_obs = zeros(n, m+p);