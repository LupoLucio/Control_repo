clear all
close all
clc

%% CONFIGURAZIONE DEL TIPO DI SEGNALE DA TRACCIARE
% Scegliere UNO dei seguenti segnali uncommentando la riga corrispondente:

% OPZIONE 1: STEP (segnale costante)
% - Polinomio caratteristico: p(s) = s
% - Dimensione esosistema: 1 stato
signal_type = 'step_sine';

% OPZIONE 2: RAMPA (crescita lineare)
% - Polinomio caratteristico: p(s) = s^2
% - Dimensione esosistema: 2 stati
% signal_type = 'ramp';

% OPZIONE 3: SINUSOIDE (frequenza omega_0)
% - Polinomio caratteristico: p(s) = s^2 + omega_0^2
% - Dimensione esosistema: 2 stati
% signal_type = 'sine';

% OPZIONE 4: STEP + SINUSOIDE
% - Polinomio caratteristico: p(s) = s^3 + omega_0^2*s
% - Dimensione esosistema: 3 stati
% signal_type = 'step_sine';

% OPZIONE 5: RAMPA + SINUSOIDE
% - Polinomio caratteristico: p(s) = s^2*(s^2 + omega_0^2)
% - Dimensione esosistema: 4 stati
% signal_type = 'ramp_sine';


%% PARAMETRI DEL SISTEMA E SPECIFICHE

% Tempo di assestamento richiesto (5%)
ts = 0.2;  % secondi

% Frequenza del segnale sinusoidale
omega_0 = 6;  % rad/s

% sigma = delta*omega_n
sigma = 4.6/ts;

% Considero damping delta = 1/sqrt(2)
delta = 1/(sqrt(2));

%Calcolo omega_n
omega_n = sigma/delta;


%% DEFINIZIONE DELLA PIANTA

% Matrici state-space del sistema
A = [0, 1; 
     0, -20];
B = [0; 
     60];
C = [1, 0];
D = 0;


%% DEFINIZIONE DELL'ESOSISTEMA IN BASE AL SEGNALE SCELTO

switch lower(signal_type)
    
    case 'step'
        % STEP COSTANTE

        % Polinomio: p(s) = s

        % Ordine del polinomio
        m = 1;

        % Coefficienti del polinomio caratteristico
        % p(s) = s + 0
        alpha = [1, 0];

        % Matrice esosistema
        A_r = [0];
        
    case 'ramp'
        % RAMPA LINEARE

        % Polinomio: p(s) = s^2

        % Ordine del polinomio
        m = 2;

        % Coefficienti del polinomio caratteristico
        % p(s) = s^2 + 0*s + 0
        alpha = [1, 0, 0];
        
        % Matrice esosistema
        A_r = [0, 1;
               0, 0];
        
    case 'sine'
        % SINUSOIDE PURA

        % Polinomio: p(s) = s^2 + omega_0^2

        % Ordine del polinomio
        m = 2;
        
        % Coefficienti del polinomio caratteristico
        % p(s) = s^2 + 0*s + omega_0^2
        alpha = [1, 0, omega_0^2];

        % Matrice esosistema
        A_r = [0, 1;
               -omega_0^2, 0];
        
    case 'step_sine'
        % STEP + SINUSOIDE

        % Polinomio: p(s) = s^3 + omega_0^2*s

        % Ordine del polinomio
        m = 3;

        % Coefficienti del polinomio caratteristico
        % p(s) = s^3 + 0*s^2 + omega_0^2*s + 0
        alpha = [1, 0, omega_0^2, 0];
        
        % Matrice esosistema (forma companion)
        A_r = [0, 1, 0;
               0, 0, 1;
               0, -omega_0^2, 0];

    case 'ramp_sine'
        % RAMPA + SINUSOIDE

        % Polinomio: p(s) = s^2*(s^2 + omega_0^2) = s^4 + omega_0^2*s^2
        
        % Ordine del polinomio
        m = 4;

        % Coefficienti del polinomio caratteristico
        % p(s) = s^4 + 0*s^3 + omega_0^2*s^2 + 0*s + 0
        alpha = [1, 0, omega_0^2, 0, 0];

        % Matrice esosistema (forma companion)
        A_r = [0, 1, 0, 0;
               0, 0, 1, 0;
               0, 0, 0, 1;
               0, 0, -omega_0^2, 0];
 
    otherwise
        error('Tipo di segnale non riconosciuto!');
end


%% COSTRUZIONE DEL SISTEMA AUMENTATO NELLO SPAZIO DEGLI ERRORI

% Dimensione stato pianta
n = size(A, 1);

% Costruzione matrice A_z del sistema aumentato
A_z = [A_r, [zeros(m-1, n); C]; 
       zeros(n, m), A];

% Costruzione matrice B_z del sistema aumentato
B_z = [zeros(m, 1); B];



%% POSIZIONAMENTO DEI POLI

% Calcolo poli desiderati in base all'ordine del sistema aumentato
total_order = m + n;

switch total_order
    case 3  % step (1) + pianta (2)
        % 2 poli complessi coniugati + 1 polo reale
        delta_1 = 1/sqrt(2);
        phi_1 = atan((sqrt(1-delta_1^2))/(delta_1));

        p1 = omega_n * exp(1j * (-pi + phi_1));
        p2 = conj(p1);
        p3 = -omega_n;
        e_poles = [p1, p2, p3];
        
    case 4  % ramp o sine (2) + pianta (2)
        % 2 coppie di poli complessi coniugati
        delta_1 = 1/sqrt(2);
        delta_2 = sqrt(3)/2;
        phi_1 = atan((sqrt(1-delta_1^2))/(delta_1));
        phi_2 = atan((sqrt(1-delta_2^2))/(delta_2));

        p1 = omega_n * exp(1j * (-pi + phi_1));
        p2 = conj(p1);
        p3 = omega_n * exp(1j * (-pi + phi_2));
        p4 = conj(p3);
        e_poles = [p1, p2, p3, p4];
        
    case 5  % step+sine (3) + pianta (2)
        % 2 coppie di poli complessi coniugati + 1 polo reale
        delta_1 = 1/sqrt(2);
        delta_2 = sqrt(3)/2;
        phi_1 = atan((sqrt(1-delta_1^2))/(delta_1));
        phi_2 = atan((sqrt(1-delta_2^2))/(delta_2));

        p1 = omega_n * exp(1j * (-pi + phi_1));
        p2 = conj(p1);
        p3 = omega_n * exp(1j * (-pi + phi_2));
        p4 = conj(p3);
        p5 = -omega_n;
        e_poles = [p1, p2, p3, p4, p5];
        
    case 6  % ramp+sine (4) + pianta (2)
        % 3 coppie di poli complessi coniugati
        delta_1 = 1/sqrt(2);
        delta_2 = sqrt(3)/2;
        delta_3 = 0.9;
        phi_1 = atan((sqrt(1-delta_1^2))/(delta_1));
        phi_2 = atan((sqrt(1-delta_2^2))/(delta_2));
        phi_3 = atan((sqrt(1-delta_3^2))/(delta_3));

        p1 = omega_n * exp(1j * (-pi + phi_1));
        p2 = conj(p1);
        p3 = omega_n * exp(1j * (-pi + phi_2));
        p4 = conj(p3);
        p5 = omega_n * exp(1j * (-pi + phi_3));
        p6 = conj(p5);
        e_poles = [p1, p2, p3, p4, p5, p6];
end


%% CALCOLO DEL GUADAGNO DI FEEDBACK

% Calcolo guadagno con pole placement
K_z = place(A_z, B_z, e_poles);


%% ESTRAZIONE DEI GUADAGNI DEL CONTROLLORE

% Guadagni per il compensatore H(s)
k_H = K_z(1:m);

% Guadagno state-feedback per la pianta
K_xi = K_z(m+1:end);

% Costruzione del compensatore H(s)
numH = fliplr(k_H);  % flip per avere coefficienti in ordine decrescente di s
denH = alpha;        % coefficienti del polinomio caratteristico


%% FILTRO DERIVATIVO PER LA STIMA DELLA VELOCITÀ

% Parametri del filtro derivativo del secondo ordine
omega_filter = 10*omega_n;  % Frequenza naturale del filtro
delta_filter = 1/sqrt(2); % Smorzamento del filtro

% Funzione di trasferimento del filtro derivativo
num_filter = [omega_filter^2, 0];
den_filter = [1, 2*delta_filter*omega_filter, omega_filter^2];
