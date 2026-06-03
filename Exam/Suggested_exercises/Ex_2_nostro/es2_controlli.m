clear all;
close all;
clc;

%% =========================================================
%  STEP 1 - Parametri di progetto (formulario eq. 1, 2, 5)
% ==========================================================
Mp = 0.2;
tr = 0.25;

delta = log(1/Mp) / sqrt(pi^2 + log(1/Mp)^2);
wn = 1.8 / tr;

fprintf('=== Parametri di progetto ===\n')
fprintf('delta = %.4f\n', delta)
fprintf('wn    = %.4f rad/s\n', wn)

%% =========================================================
%  STEP 2 - Scelta del tempo di campionamento Ts
% ==========================================================
Ts = 0.02;
fprintf('\n=== Discretizzazione (ZOH - Tabella 2, colonna Exact) ===\n')
fprintf('Ts = %.4f s\n', Ts)

%% =========================================================
%  STEP 3 - Discretizzazione esatta (ZOH)
% ==========================================================
s = tf('s');
sys_c = 90/(s^2+60*s+3);
sys_d = c2d(sys_c, Ts, 'zoh');
[Phi, Gamma, H_mat, J] = ssdata(sys_d);

disp('Phi =');   disp(Phi)
disp('Gamma ='); disp(Gamma)
disp('H (C matrix) ='); disp(H_mat)

%% =========================================================
%  STEP 4 - Sistema aumentato Error Space
%           Formulario Eq. (12)
%           Stato esteso: z_e = [x; xi]
%
%           Ae = [ Phi,    0 ]     Be = [ Gamma ]
%                [ -H_mat, 1 ]          [   0   ]
% ==========================================================
n = size(Phi, 1); % ordine pianta = 2

Ae = [Phi,        zeros(n, 1);
      -H_mat,     1          ];

Be = [Gamma;
      0    ];

fprintf('\n=== Sistema aumentato Error Space (eq. 12) ===\n')
disp('Ae ='); disp(Ae)
disp('Be ='); disp(Be)

%% =========================================================
%  STEP 5 - Poli desiderati del controllore
% ==========================================================
s_dom = [-delta*wn + 1j*wn*sqrt(1-delta^2);
         -delta*wn - 1j*wn*sqrt(1-delta^2)];

s_extra = 4 * (-delta * wn); % Polo integratore piu' veloce dei dominanti
s_all = [s_dom; s_extra];
z_all = exp(s_all * Ts);

fprintf('\n=== Poli desiderati controllore ===\n')
fprintf('Poli dominanti: %.4f +/- %.4fj\n', real(s_dom(1)), imag(s_dom(1)))
fprintf('Polo integratore: %.4f\n', s_extra)
fprintf('Poli in z:\n'); disp(z_all)

%% =========================================================
%  STEP 8 - Costruzione H(s) per Simulink (formulario eq. 14)
%           H(s) = -(k_{m-1}*s^{m-1} + ... + k_0) / alpha(s)
%           Per il gradino (m=1): H(s) = -k_0 / s = -Ki / s
% ==========================================================
signal_type = 'step';
omega = 6;

switch signal_type
    case 'step'
        m = 2;
        alpha = [1, 0, 0];          % p(s) = s
    case 'ramp'
        m = 3;
        alpha = [1, 0, 0, 0];       % p(s) = s^2
    case 'sine'
        m = 3;
        alpha = [1, 0, omega^2, 0]; % p(s) = s^2 + omega^2
    case 'step_sine'
        m = 4;
        alpha = [1, 0, omega^2, 0, 0]; % p(s) = s^3 + omega^2*s
    otherwise
        error('Tipo di segnale non supportato!');
end

%% =========================================================
%  STEP 6 - Calcolo guadagno controllore Kz
%           u[k] = -Kz * [x_hat[k]; xi[k]]
%                = -[Kx, Ki] * [x_hat[k]; xi[k]]
% ==========================================================
Kz = place(Ae, Be, z_all);

kx   = Kz(1:m);
Kxi  = Kz(m+1:end);
numH = -fliplr(kx);
denH = alpha;
eig_ctrl = eig(Ae - Be * Kz);

%% =========================================================
%  STEP 7 - Full-order state observer (solo pianta)
%
%  Il full-order observer stima x[k] dalla misura y[k]=H_mat*x[k].
%  Lo stato xi NON va osservato: si calcola direttamente come
%      xi[k+1] = xi[k] + r[k] - y[k]
%  quindi L si progetta su (Phi, H_mat) soltanto.
%
%  Dinamica dell'osservatore:
%      x_hat[k+1] = Phi * x_hat[k] + Gamma * u[k] + L * (y[k] - H_mat * x_hat[k])
%                 = (Phi - L*H_mat) * x_hat[k] + Gamma * u[k] + L * y[k]
%
%  Regola: poli osservatore 3-5x piu' veloci dei dominanti (in continuo)
%          poi mappati in z con z = exp(s*Ts)
% ==========================================================
s_obs = 4 * s_dom;        % 4x piu' veloci in continuo
z_obs = exp(s_obs * Ts);  % mappati in z

L = place(Phi', H_mat', z_obs)';

fprintf('\n=== Full-order State Observer ===\n')
fprintf('Poli osservatore in z:\n'); disp(z_obs)
disp('L ='); disp(L)

eig_obs = eig(Phi - L * H_mat);
fprintf('Autovalori osservatore:\n'); disp(eig_obs)
fprintf('Stabilita'': %d\n', all(abs(eig_obs) < 1))

% =========================================================
%  Matrici dell'osservatore per Simulink
%  Il blocco "Discrete State-Space" in Simulink implementa:
%      x_hat[k+1] = A_obs * x_hat[k] + B_obs * [u[k]; y[k]]
%      x_out[k]   = C_obs * x_hat[k] + D_obs * [u[k]; y[k]]
%
%  Dalla dinamica dell'osservatore:
%      x_hat[k+1] = (Phi - L*H_mat)*x_hat[k] + [Gamma, L]*[u[k]; y[k]]
%      x_out[k]   =  I * x_hat[k]  +  0 * [u[k]; y[k]]
% ==========================================================
A_obs = Phi - L * H_mat;           % n x n
B_obs = [Gamma, L];                % n x 2  (ingressi: u e y)
C_obs = eye(n);                    % n x n  (uscita: tutto x_hat)
D_obs = zeros(n, 2);               % n x 2

obsv = ss(A_obs, B_obs, C_obs, D_obs);
tf_obsv = tf(obsv);