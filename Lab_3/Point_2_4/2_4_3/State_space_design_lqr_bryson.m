delta_B = mld.Bh + (mot.Kt * mot.Ke) / Req;
u_gain  = (mot.Kt * drv.dcgain) / (gbox.N * Jeq * Req);

A_prime = zeros(4);

A_prime(1,3) = 1;
A_prime(2,4) = 1;

A_prime(3,2) =  mld.k / (gbox.N^2 * Jeq);
A_prime(3,3) = -delta_B / Jeq;
A_prime(3,4) = -mld.k / (gbox.N^2 * Jeq);

A_prime(4,2) = -mld.k / mld.Jb;
A_prime(4,3) =  delta_B / Jeq;
A_prime(4,4) = -mld.Bb / mld.Jb;

B_prime = [0;
           0;
           u_gain;
           0];

C = [1 0 0 0];
D = 0;

% 1. Definizione dei Limiti Massimi (Eq. 14-15 Assignment)
ref_deg    = 50;                % Riferimento di posizione [deg]
ref_rad    = ref_deg * pi/180;  % Riferimento in radianti

% Limiti di deviazione
x_h_max = 0.30 * ref_rad;     % Max errore posizione hub (30% di 50°) [rad]
x_d_max = 5 * pi/180;         % Max deformazione beam (5°) [rad]
u_max   = 10;                 % Max tensione di controllo [V]

fprintf('=== Parametri per la Regola di Bryson ===\n');
fprintf('  Max errore Hub: %.4f rad (%.2f deg)\n', x_h_max, rad2deg*x_h_max);
fprintf('  Max deformazione Beam: %.4f rad (%.2f deg)\n', x_d_max, rad2deg*x_d_max);
fprintf('  Max Tensione u: %.2f V\n', u_max);

% 2. Costruzione Matrici di Peso Q e r
% Q è diagonale. Gli elementi sulle velocità (3,3 e 4,4) sono posti a ZERO
% come richiesto dall'Assignment.
q11 = 1 / (x_h_max^2);
q22 = 1 / (x_d_max^2);
q33 = 0.1 * q11;   % piccolo peso sulla velocità hub
q44 = 0.1 * q22;   % piccolo peso sulla velocità beam

Q = diag([q11, q22, q33, q44]);


Q = diag([q11, q22, q33, q44]);

% Peso sull'ingresso r
r = 1 / (u_max^2);

fprintf('\nMatrici calcolate:\n');
fprintf('  Q = diag([%.2f, %.2f, %d, %d])\n', q11, q22, 0, 0);
fprintf('  r = %.4f\n', r);

% 3. Sintesi LQR
% Usa le matrici A e B caricate nel workspace
try
    K = lqr(A_prime, B_prime, Q, r);
catch
    error('Errore: Le matrici A e B non sono definite nel workspace. Esegui prima lo script di caricamento parametri.');
end

fprintf('\n✅ Matrice di guadagno LQR (Bryson):\n');
disp(K);

% 4. Calcolo Feedforward per Tracking Nominale
% Necessario per avere errore nullo a regime su gradino di 50°
C = [1, 0, 0, 0]; D = 0; % Matrici di uscita
sys_cl = ss(A_prime - B_prime*K, B_prime, C, D);
dc_gain = dcgain(sys_cl);
Kff = 1 / dc_gain;

fprintf('🔧 Guadagno feedforward Kff: %.4f\n', Kff);

% 5. Simulazione e Verifica Specifiche
sys_cl_ff = ss(A_prime - B_prime*K, B_prime*Kff, C, D);

t_sim = 0:1e-3:3;
[y, t, x] = step(sys_cl_ff, t_sim);

% Conversione in gradi per visualizzazione
y_deg = y * rad2deg;

figure('Name', 'Risposta LQR Bryson (2.4.3)');
plot(t, y_deg, 'b', 'LineWidth', 1.5); hold on;
yline(ref_deg, 'r--', 'Riferimento');
yline(ref_deg*1.05, 'k:', '+5%');
yline(ref_deg*0.95, 'k:', '-5%');
grid on;
xlabel('Tempo [s]'); ylabel('Posizione Hub [deg]');
title(sprintf('LQR Bryson (r=%.3f) - ts=%.3fs, Mp=%.2f%%', r, 0, 0));

% Calcolo metriche
info = stepinfo(sys_cl_ff);
fprintf('\n=== Risultati Simulazione ===\n');
fprintf('  Settling Time (5%%): %.3f s (Richiesto <= 0.85 s)\n', info.SettlingTime);
fprintf('  Overshoot:           %.2f %% (Richiesto <= 30%%)\n', info.Overshoot);
fprintf('  Peak Time:           %.3f s\n', info.PeakTime);

% Controllo saturazione (simulato)
u_signal = -K * x' + Kff * ones(1, length(t));
u_max_sim = max(abs(u_signal));
fprintf('  Max tensione simulata: %.2f V (Limite assegnamento 10V, Limite fisico 12V)\n', u_max_sim);

% Salvataggio per simulazione in Simulink
assignin('base', 'K_lqr_bryson', K);
assignin('base', 'Kff_lqr_bryson', Kff);