%% Esercizio 2: Controllo Discreto con Criteri di Progetto Espliciti
clear all; close all; clc;

%% 1. Definizione Pianta e Scelta del Campionamento
num = 90;
den = [1, 60, 3];
sys_c = tf(num, den);

tr_spec = 0.25;
Mp_spec = 0.2;

% CRITERIO 1: Ts <= tr / 10 (Regola pratica campionamento)
Ts = tr_spec / 10; 
fprintf('1. Tempo di campionamento scelto: Ts = %.3f s (Regola: tr/10)\n', Ts);

% Discretizzazione Esatta (ZOH) - Tabella 2 del Formulario
sys_d = c2d(sys_c, Ts, 'zoh');
[Phi, Gamma, H, J] = ssdata(sys_d);

%% 2. Calcolo Parametri Dominanti dal Dominio del Tempo
% Formule (1) e (5) del Formulario
wn = 1.8 / tr_spec; 
delta = log(1/Mp_spec) / sqrt(pi^2 + log(1/Mp_spec)^2);

% Poli continui dominanti desiderati
s_1 = -delta*wn + 1j*wn*sqrt(1-delta^2);
s_2 = conj(s_1);

% Mappatura esatta nel piano Z
z_1 = exp(s_1 * Ts);
z_2 = exp(s_2 * Ts);
fprintf('2. Poli dominanti del controllore: z = %.4f %+.4fi\n', real(z_1), imag(z_1));

%% 3. Spazio degli Errori (Azione Integrale)
% CRITERIO: Estensione dello stato per inseguimento asintotico a gradino
% xi[k+1] = xi[k] + e[k] = xi[k] + r[k] - Cd*x[k]
Ae = [Phi, zeros(2,1); 
      -H, 1];
Be = [Gamma; 0];

%% 4. Scelta del 3° Polo (Integratore)
% CRITERIO 3: Regola della Dominanza. 
% Il polo dell'integratore deve essere 4 volte più veloce (parte reale) 
% dei poli dominanti per non influenzare il transitorio, ma non troppo 
% da evitare sforzi di controllo eccessivi.
speed_factor_integrator = 4;
s_3 = speed_factor_integrator * real(s_1); % Parte reale * 4 (più negativo)
z_3 = exp(s_3 * Ts); % Mappatura in Z

p_ctrl = [z_1, z_2, z_3];
fprintf('3. Polo integratore scelto: z_3 = %.4f (Regola: 4x più veloce in continuo)\n', z_3);

% Calcolo guadagno K
K = place(Ae, Be, p_ctrl);

%% 5. Scelta Poli dell'Osservatore
% CRITERIO 4: Regola del Fattore di Velocità.
% Il modulo dei poli dell'osservatore deve essere circa 1/3 del modulo 
% dei poli dominanti del controllore per garantire una stima rapida 
% senza amplificare eccessivamente il rumore (guadagni L troppo alti).
r_ctrl = abs(z_1); % Modulo dei poli dominanti
speed_factor_observer = 3;
r_obs_target = r_ctrl / speed_factor_observer;

% Per evitare l'errore di molteplicità di 'place' in sistemi SISO, 
% scegliamo due poli reali distinti vicini al modulo target.
z_obs_1 = r_obs_target;      
z_obs_2 = r_obs_target + 0.05; % Leggera separazione per stabilità numerica

p_obs = [z_obs_1, z_obs_2];
fprintf('4. Poli osservatore scelti: z = %.4f, %.4f (Regola: modulo ~1/3 di |z_dominante|=%.4f)\n', z_obs_1, z_obs_2, r_ctrl);

% Calcolo guadagno L (usando la dualità)
L = place(Phi', H', p_obs)'; 

%% 6. Riepilogo e Verifica
fprintf('\n--- GUADAGNI CALCOLATI ---\n');
fprintf('K = [%.3f, %.3f, %.3f]  (Kx1, Kx2, Ki)\n', K(1), K(2), K(3));
fprintf('L = [%.3f; %.3f]\n', L(1), L(2));

fprintf('\n--- VERIFICA AUTOVALORI ---\n');
fprintf('Poli sistema esteso controllato (Ae - Be*K):\n');
disp(eig(Ae - Be*K));

fprintf('Poli dinamica errore osservatore (Ad - L*Cd):\n');
disp(eig(Phi - L*H));