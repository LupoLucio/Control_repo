% 1. Parametri di Specifica
ts = 0.85;   % Settling time massimo [s]
Mp = 0.30;   % Overshoot massimo [adimensionale]

% Calcolo parametri geometrici della regione ammissibile
delta_val = log(1/Mp) / sqrt(pi^2 + log(1/Mp)^2);
wn        = 3 / (delta_val * ts);
phi       = atan(sqrt(1 - delta_val^2) / delta_val);
sigma_min = -3 / ts;  % Parte reale massima ammissibile (verticale)

fprintf('📐 Geometria Regione Ammissibile:\n');
fprintf('   σ <= %.2f rad/s\n', sigma_min);
fprintf('   Angolo φ  = %.2f° (Smorzamento δ = %.3f)\n', rad2deg*phi, delta_val);

% 2. Costruzione Modelli per SRL
% L'Assignment richiede C = [1 0 0 0] per pesare solo la posizione dell'hub
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

% Modello normale G(s)
sysG = ss(A_prime, B_prime, C, D);
% Modello "specchio" G(-s) richiesto per il SRL
sysGp = ss(-A_prime, -B_prime, C, D);

% La funzione di loop per il SRL è L(s) = G(-s)G(s)
% Il luogo delle radici è tracciato rispetto al guadagno k = 1/r
sys_loop = sysGp * sysG;

% 3. Plot del Symmetric Root Locus
figure('Name', 'Symmetric Root Locus (SRL) Design', 'Color', 'w');

% Tracciamo il luogo delle radici
% Nota: rlocus traccia rispetto a k. Qui k = 1/r.
[k_vals, poles] = rlocus(sys_loop);

hold on; grid on;

% --- Disegno della Regione Ammissibile ---
% Linea verticale σ = sigma_min
plot([sigma_min, sigma_min], [-80, 80], 'r--', 'LineWidth', 1.5, 'DisplayName', 'Limite ts (σ)');

% Linee oblique ±φ (smorzamento Mp)
% Partono dall'origine (0,0)
line_re = [-2, 0];
line_im_up = [-2 * tan(phi), 0];
line_im_dw = [2 * tan(phi), 0];

plot(line_re, line_im_up, 'r:', 'LineWidth', 1.5, 'DisplayName', 'Limite Mp (+φ)');
plot(line_re, line_im_dw, 'r:', 'LineWidth', 1.5, 'DisplayName', 'Limite Mp (-φ)');

% Evidenziamo la zona ammissibile (triangolo verso sinistra)
x_fill = [-80, sigma_min, sigma_min];
y_fill = [80, 80*tan(phi), -80*tan(phi)];
patch(x_fill, y_fill, 'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', 'Regione Ammissibile');

plot(poles, 'bx', 'MarkerSize', 4);
title('Symmetric Root Locus: Scelta del peso r (guadagno plot = 1/r)');
xlabel('Real Axis'); ylabel('Imaginary Axis');
legend('Location', 'Best');
xlim([-60, 10]); ylim([-60, 60]);

% 4. Scelta del peso r (Trial & Error o Analisi Grafica)
% Il guadagno k nel plot di rlocus corrisponde a 1/r.
% Dobbiamo scegliere k (e quindi r = 1/k) tale che i poli siano nella zona verde.

% Esempio: proviamo a trovare un k che metta i poli circa a sigma_min
% Possiamo usare rlocfind per cliccare sul grafico e scegliere k, 
% oppure calcolarlo per tentativi. Qui propongo un valore tipico.

% Supponiamo di voler i poli dominanti con parte reale = -4.5 (più veloce di -3.53)
% Calcoliamo la K associata a un certo k (es. k=100) per vedere dove siamo
test_k = 1e+07; 
test_r = 1/test_k;

fprintf('\n🎯 Scelta peso r:\n');
fprintf('   Proviamo r = %.2e (guadagno k = %.2f)\n', test_r, test_k);

% Calcolo LQR con il r scelto
% Usiamo lqry perché la penalità è sull'uscita y = C*x (var_theta_h^2)
K = lqry(sysG, 1, test_r);

% Verifica autovalori
sys_cl = ss(A_prime - B_prime*K, B_prime, C, D);
closed_poles = eig(sys_cl);

fprintf('   Poli anello chiuso: \n');
disp(closed_poles);

% Controllo se sono nella regione
max_real = max(real(closed_poles));
fprintf('   Max(Parte Reale) = %.2f (Richiesto <= %.2f) -> %s\n', ...
    max_real, sigma_min, string(max_real <= sigma_min));

% Plot dei poli calcolati sul grafico
plot(real(closed_poles), imag(closed_poles), 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Poli scelti (r attuale)');

% 5. Calcolo Feedforward Kff per Tracking Nominale
% L'Assignment richiede "nominal perfect tracking"
% Anche con LQR serve il pre-filtro Kff se non c'è azione integrale
sys_cl_nom = ss(A_prime - B_prime*K, B_prime, C, D);
dc_gain = dcgain(sys_cl_nom);
Kff = 1 / dc_gain;

fprintf('\n✅ Risultato Finale:\n');
fprintf('   Matrice K = [%.4f, %.4f, %.4f, %.4f]\n', K);
fprintf('   Feedforward Kff = %.4f\n', Kff);

% Salvataggio nel workspace
assignin('base', 'K_lqr', K);
assignin('base', 'Kff_lqr', Kff);