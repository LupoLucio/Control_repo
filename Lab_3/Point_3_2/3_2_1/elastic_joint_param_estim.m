%% 3.2 - STIMA DEI PARAMETRI DEL GIUNTO ELASTICO (SLdRT)
% Procedura descritta nel paragrafo 4 dell'Handout
% Implementazione per acquisizione dati tramite Simulink Desktop Real-Time

%% CONFIGURAZIONE ACQUISIZIONE DATI SLdRT
% Il modello Simulink deve essere configurato per:
% 1. Bloccare l'hub (thh_ref = 0)
% 2. Acquisire theta_d (beam displacement)
% 3. Campionamento a Ts = 1 ms (o come configurato)

fprintf('\n📋 ISTRUZIONI PER L''ACQUISIZIONE:\n');
fprintf('   1. Assicurarsi che l''hub sia bloccato (thh_ref = 0)\n');
fprintf('   2. Deflettere il beam di max 20° dalla posizione di equilibrio\n');
fprintf('   3. Rilasciare il beam e acquisire la risposta libera\n');
fprintf('   4. Durata acquisizione consigliata: 2-3 secondi\n');
fprintf('   5. Campionamento: Ts = 1 ms\n\n');

%% CARICAMENTO DATI SPERIMENTALI DA SLdRT
% I dati devono essere salvati dal modello Simulink SLdRT nel workspace
% Variabili attese:
%   - t: vettore dei tempi [s]
%   - theta_d: beam displacement [deg] o [rad]
%
% Esempio di caricamento (da adattare al proprio modello):
% load('beam_response_data.mat');  % Se i dati sono stati salvati su file
% OPPURE i dati sono già nel workspace dopo l'esecuzione del modello SLdRT

%% PRE-ELABORAZIONE DATI
% Conversione in radianti se necessario
if max(abs(theta_d)) > 1  % Se i valori sono in gradi
    theta_d = theta_d * deg2rad;
    fprintf('   ✓ Dati convertiti da gradi a radianti\n');
end

% Rimozione bias (media sugli ultimi campioni a regime)
bias_est = mean(theta_d(end-100:end));
theta_d = theta_d - bias_est;
fprintf('   ✓ Bias rimosso: %.4f deg\n', bias_est*rad2deg);

%% 1) INDIVIDUAZIONE DEI PICCHI DELLA RISPOSTA
% Calcolo del valore assoluto per individuare tutti i picchi
theta_d_abs = abs(theta_d);

% Trova i picchi locali
[pks, locs] = findpeaks(theta_d_abs, t, ...
    'MinPeakDistance', 0.05, ...           % Distanza minima tra picchi [s]
    'MinPeakHeight', 0.05*max(theta_d_abs));  % Altezza minima (5% del max)

fprintf('\n🔍 Picchi individuati: %d\n', length(pks));

if length(pks) < 3
    error('❌ Numero insufficiente di picchi per la stima (minimo 3). Controllare i dati.');
end

% Visualizza i picchi trovati
fprintf('   Posizioni temporali dei picchi:\n');
for i = 1:min(5, length(locs))
    fprintf('      Picco %d: t = %.4f s, |θd| = %.4f deg\n', ...
        i, locs(i), pks(i)*rad2deg);
end
if length(locs) > 5
    fprintf('      ... e altri %d picchi\n', length(locs)-5);
end

%% 2) STIMA DEL FATTORE DI SMORZAMENTO (δ) CON METODO DEL DECREMENTO LOGARITMICO
% Il decremento logaritmico ξ è la pendenza della retta che interpola 
% (k, log|ϑd(tk)|) - eq. (71) dell'Handout

M = length(pks);  % Numero di picchi
k_vec = 0:(M-1);
log_pks = log(pks);

% Least Squares fitting: z = -a*k + b
% dove a = ξ (decremento logaritmico) - eq. (73)-(79)
Phi = [-k_vec', ones(M,1)];  % Matrice dei regressori
theta_LS = (Phi' * Phi) \ (Phi' * log_pks');  % Soluzione LS - eq. (79)

a_hat = theta_LS(1);  % Pendenza (decremento logaritmico ξ)
b_hat = theta_LS(2);  % Intercetta

xi_hat = a_hat;  % Stima del decremento logaritmico

% Calcolo del fattore di smorzamento δ dalla eq. (80) dell'Handout
delta_hat = xi_hat / sqrt(pi^2 + xi_hat^2);

fprintf('\n📐 Stima fattore di smorzamento:\n');
fprintf('   Decremento logaritmico ξ = %.4f\n', xi_hat);
fprintf('   Fattore di smorzamento δ = %.4f\n', delta_hat);
fprintf('   (Valore tipico atteso: 0.02-0.1)\n');

% Verifica coerenza
if delta_hat < 0.01 || delta_hat > 0.2
    warning('⚠️  Valore di δ fuori dall''intervallo tipico. Verificare i dati.');
end

%% 3) STIMA DELLA FREQUENZA NATURALE (ωn)
% Calcolo degli intervalli di tempo tra picchi consecutivi
T_intervals = diff(locs);  % Tk = t(k+1) - tk

% Stima della frequenza ω da ogni intervallo (eq. 81)
omega_k = pi ./ T_intervals;  % ωk = π/Tk

% Media delle stime (eq. 82)
omega_hat = mean(omega_k);
omega_std = std(omega_k);

% Calcolo della frequenza naturale ωn (eq. 83)
omega_n_hat = omega_hat / sqrt(1 - delta_hat^2);

fprintf('\n📐 Stima frequenza naturale:\n');
fprintf('   Frequenza oscillazione ω = %.4f rad/s (±%.4f)\n', omega_hat, omega_std);
fprintf('   Frequenza naturale ωn = %.4f rad/s\n', omega_n_hat);
fprintf('   Frequenza naturale fn = %.2f Hz\n', omega_n_hat/(2*pi));
fprintf('   (Valore tipico atteso: 20-30 rad/s ≈ 3-5 Hz)\n');

%% 4) CALCOLO DEI PARAMETRI FISICI (eq. 33)
% Stima del coefficiente di attrito viscoso del beam
Bb_hat = mld.Jb * (2 * delta_hat * omega_n_hat);

% Stima della rigidità del giunto elastico
k_hat = mld.Jb * omega_n_hat^2;

fprintf('\n⚙️ Parametri fisici stimati:\n');
fprintf('   Beam viscous friction Bb = %.6f Nm/(rad/s)\n', Bb_hat);
fprintf('   Joint stiffness k        = %.4f Nm/rad\n', k_hat);

%% 5) CONFRONTO CON VALORI TENTATIVI
% Valori tentativi usati nelle simulazioni (Sec. 2.1 Assignment)
Bb_tentative = 3.4e-3;
k_tentative = 0.83;

fprintf('\n📊 Confronto con valori tentativi:\n');
fprintf('   Bb: %.6f vs %.6f (diff: %.1f%%)\n', ...
    Bb_hat, Bb_tentative, 100*(Bb_hat-Bb_tentative)/Bb_tentative);
fprintf('   k:  %.4f vs %.4f (diff: %.1f%%)\n', ...
    k_hat, k_tentative, 100*(k_hat-k_tentative)/k_tentative);

%% 6) VALIDAZIONE DEL MODELLO STIMATO
% Simulazione della risposta con i parametri stimati
sigma_hat = -delta_hat * omega_n_hat;
omega_hat_valid = omega_n_hat * sqrt(1 - delta_hat^2);

% Trova il primo picco per inizializzare A e phi
[~, idx_first] = findpeaks(theta_d_abs, t, 'MinPeakDistance', 0.05, 'NumPeaks', 1);
if ~isempty(idx_first)
    A_hat = pks(1) / exp(sigma_hat * locs(1));
    phi_hat = 0;  % Approssimazione
else
    A_hat = max(abs(theta_d));
    phi_hat = 0;
end

theta_d_model = A_hat * exp(sigma_hat * t) .* cos(omega_hat_valid * t + phi_hat);

% Calcolo errore
error_rms = sqrt(mean((theta_d - theta_d_model).^2));
error_max = max(abs(theta_d - theta_d_model));

fprintf('\n✅ Validazione modello stimato:\n');
fprintf('   RMS error: %.6f rad (%.4f deg)\n', error_rms, error_rms*rad2deg);
fprintf('   Max error: %.6f rad (%.4f deg)\n', error_max, error_max*rad2deg);

%% 7) PLOT RISULTATI
figure('Name', 'Stima Parametri Giunto Elastico', 'Color', 'w', 'Position', [100, 100, 1200, 800]);

% Plot 1: Risposta sperimentale con picchi evidenziati
subplot(2, 2, 1);
plot(t, theta_d*rad2deg, 'b-', 'LineWidth', 1.5); hold on;
plot(locs, pks*rad2deg, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
grid on;
xlabel('Tempo [s]');
ylabel('\theta_d [deg]');
title('Risposta Libera del Beam con Picchi Individuati');
legend('Dati sperimentali', 'Picchi', 'Location', 'Best');

% Plot 2: Decremento logaritmico
subplot(2, 2, 2);
plot(k_vec, log_pks, 'bo', 'MarkerSize', 6); hold on;
plot(k_vec, -xi_hat*k_vec + b_hat, 'r-', 'LineWidth', 2);
grid on;
xlabel('Numero picco k');
ylabel('log(|\theta_d(t_k)|)');
title(['Decremento Logaritmico: \xi = ' num2str(xi_hat, '%.4f')]);
legend('Dati', 'Fit LS', 'Location', 'Best');

% Plot 3: Confronto modello vs dati
subplot(2, 2, 3);
plot(t, theta_d*rad2deg, 'b-', 'LineWidth', 1.5); hold on;
plot(t, theta_d_model*rad2deg, 'r--', 'LineWidth', 2);
grid on;
xlabel('Tempo [s]');
ylabel('\theta_d [deg]');
title('Confronto Modello Stimato vs Dati');
legend('Dati sperimentali', 'Modello stimato', 'Location', 'Best');

% Plot 4: Errore di modellazione
subplot(2, 2, 4);
plot(t, (theta_d - theta_d_model)*rad2deg, 'k-', 'LineWidth', 1.5);
grid on;
xlabel('Tempo [s]');
ylabel('Errore [deg]');
title(['Errore di Modellazione (RMS = ' num2str(error_rms*rad2deg, '%.3f') ' deg)']);