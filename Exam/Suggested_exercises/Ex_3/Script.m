clear all
close all
clc

%% System Definition
% Dynamic model
A = [0, 1; -2, -40];
B = [0; 40];
C = [1, 0];
D = 0;

n = size(A, 1);   % ordine sistema = 2
p = size(C, 1);   % numero uscite  = 1
m = size(B, 2);   % numero ingressi = 1

P_ss_c = ss(A,B,C,D);

Ts = 0.01e-3;

P_ss_d = c2d(P_ss_c,Ts,'zoh');

[Phi, Gamma, H, J] = ssdata(P_ss_d);

Phi_e = [1, H; zeros(n,1), Phi];
Gamma_e = [zeros(p,m);Gamma];
H_e = [zeros(p,p),H];
J_e = J;


%% Optimal Feedback Controller
% Cost matrices from Bryson's rule
Q = diag(ones(1, n+1) / 10);   % (n+1) x (n+1): n stati + 1 integratore
R = 1/20;


% Feedback gain and resulting closed loop poles
[Ke,~,poles_T_d] = dlqr(Phi_e,Gamma_e,Q,R);
Ki = Ke(1);
K  = Ke(2:n+1);

% Nx, Nu
N = [Phi - eye(n), Gamma; H, J] \ [zeros(n, 1); 1];
Nx = N(1:n);
Nu = N(n+1:end);


%% Observer
% eigenvalues in z domain, 4-5 times faster than CL
poles_d_obs = [0.2,0.25];

% Observer Gain
L = place(Phi',H',poles_d_obs)';

Phi_obs = Phi-L*H;
Gamma_obs = [Gamma, L];
H_obs = eye(n);
J_obs = zeros(n, m+p);