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

% Frequenza del segnale sinusoidale (se applicabile)
omega_0 = 6;  % rad/s

% Calcolo della parte reale minima dei poli per il tempo di assestamento
sigma = 4.6/ts;  % Per ts = 0.2s -> sigma = 23

% Frequenza naturale minima (considerando damping delta = 1/sqrt(2))
omega_n_min = sigma*sqrt(2);

% Frequenza naturale scelta (deve essere > omega_n_min)
omega_n = 35;  % rad/s


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
        % ODE: r_dot = 0
        % Polinomio: p(s) = s
        
        % Matrice esosistema (forma companion)
        A_r = [0];
        
        % Coefficienti del polinomio caratteristico
        % p(s) = s + 0
        alpha = [1, 0];  % [s^1, s^0]
        
        % Ordine del polinomio
        m = 1;
        
    case 'ramp'
        % RAMPA LINEARE
        % ODE: r_ddot = 0
        % Polinomio: p(s) = s^2
        
        % Matrice esosistema (forma companion)
        A_r = [0, 1;
               0, 0];
        
        % Coefficienti del polinomio caratteristico
        % p(s) = s^2 + 0*s + 0
        alpha = [1, 0, 0];  % [s^2, s^1, s^0]
        
        % Ordine del polinomio
        m = 2;
        
    case 'sine'
        % SINUSOIDE PURA
        % ODE: r_ddot + omega_0^2*r = 0
        % Polinomio: p(s) = s^2 + omega_0^2
        
        % Matrice esosistema (forma companion)
        A_r = [0, 1;
               -omega_0^2, 0];
        
        % Coefficienti del polinomio caratteristico
        % p(s) = s^2 + 0*s + omega_0^2
        alpha = [1, 0, omega_0^2];  % [s^2, s^1, s^0]
        
        % Ordine del polinomio
        m = 2;
        
    case 'step_sine'
        % STEP + SINUSOIDE
        % ODE: r^(3) + omega_0^2*r_dot = 0
        % Polinomio: p(s) = s^3 + omega_0^2*s
        
        % Matrice esosistema (forma companion)
        A_r = [0, 1, 0;
               0, 0, 1;
               0, -omega_0^2, 0];
        
        % Coefficienti del polinomio caratteristico
        % p(s) = s^3 + 0*s^2 + omega_0^2*s + 0
        alpha = [1, 0, omega_0^2, 0];  % [s^3, s^2, s^1, s^0]
        
        % Ordine del polinomio
        m = 3;
        
    case 'ramp_sine'
        % RAMPA + SINUSOIDE
        % Polinomio: p(s) = s^2*(s^2 + omega_0^2) = s^4 + omega_0^2*s^2
        
        % Matrice esosistema (forma companion)
        A_r = [0, 1, 0, 0;
               0, 0, 1, 0;
               0, 0, 0, 1;
               0, 0, -omega_0^2, 0];
        
        % Coefficienti del polinomio caratteristico
        % p(s) = s^4 + 0*s^3 + omega_0^2*s^2 + 0*s + 0
        alpha = [1, 0, omega_0^2, 0, 0];  % [s^4, s^3, s^2, s^1, s^0]
        
        % Ordine del polinomio
        m = 4;
        
    otherwise
        error('Tipo di segnale non riconosciuto!');
end


%% COSTRUZIONE DEL SISTEMA AUMENTATO NELLO SPAZIO DEGLI ERRORI

% Dimensione stato pianta
n = size(A, 1);

% Dimensione stato esosistema
n_r = size(A_r, 1);

% Costruzione matrice A_e del sistema augmentato
% Struttura: [A_r, [zeros(n_r-1,2); C]; zeros(2,n_r), A]
A_z = [A_r, [zeros(n_r-1, n); C]; 
       zeros(n, n_r), A];

% Costruzione matrice B_e del sistema aumentato
B_z = [zeros(n_r, 1); B];



%% POSIZIONAMENTO DEI POLI

% Scelta dei coefficienti di smorzamento
delta_1 = 1/sqrt(2);      % ~0.707 (buon compromesso)
delta_2 = sqrt(3)/2;      % ~0.866 (più smorzato)

% Calcolo poli desiderati in base all'ordine del sistema augmentato
total_order = n_r + n;

switch total_order
    case 3  % step (1) + pianta (2)
        % 2 poli complessi coniugati + 1 polo reale
        p1 = omega_n * exp(1j * (-pi + acos(delta_1)));
        p2 = conj(p1);
        p3 = -omega_n;
        e_poles = [p1, p2, p3];
        
    case 4  % ramp o sine (2) + pianta (2)
        % 2 coppie di poli complessi coniugati
        p1 = omega_n * exp(1j * (-pi + acos(delta_1)));
        p2 = conj(p1);
        p3 = omega_n * exp(1j * (-pi + acos(delta_2)));
        p4 = conj(p3);
        e_poles = [p1, p2, p3, p4];
        
    case 5  % step+sine (3) + pianta (2)
        % 2 coppie di poli complessi coniugati + 1 polo reale
        p1 = omega_n * exp(1j * (-pi + acos(delta_1)));
        p2 = conj(p1);
        p3 = omega_n * exp(1j * (-pi + acos(delta_2)));
        p4 = conj(p3);
        p5 = -omega_n;
        e_poles = [p1, p2, p3, p4, p5];
        
    case 6  % ramp+sine (4) + pianta (2)
        % 3 coppie di poli complessi coniugati
        p1 = omega_n * exp(1j * (-pi + acos(delta_1)));
        p2 = conj(p1);
        p3 = omega_n * exp(1j * (-pi + acos(delta_2)));
        p4 = conj(p3);
        p5 = omega_n * exp(1j * (-pi + acos(0.9)));  % molto smorzato
        p6 = conj(p5);
        e_poles = [p1, p2, p3, p4, p5, p6];
        
    otherwise
        % Generazione automatica poli per ordini superiori
        e_poles = zeros(total_order, 1);
        for i = 1:floor(total_order/2)
            delta = 0.7 + 0.1*(i-1);  % smorzamento crescente
            if delta > 0.95, delta = 0.95; end
            p = omega_n * exp(1j * (-pi + acos(delta)));
            e_poles(2*i-1) = p;
            e_poles(2*i) = conj(p);
        end
        if mod(total_order, 2) ~= 0
            e_poles(end) = -omega_n;
        end
end


%% CALCOLO DEL GUADAGNO DI FEEDBACK

% Calcolo guadagno con pole placement
K_z = place(A_z, B_z, e_poles);


%% ESTRAZIONE DEI GUADAGNI DEL CONTROLLORE

% Il vettore k_e contiene:
% - primi m elementi: guadagni per le derivate dell'errore [e, e_dot, ...]
% - ultimi n elementi: guadagni per lo stato della pianta [x1, x2]

% Guadagni per il compensatore H(s)
k_H = K_z(1:m);

% Guadagno state-feedback per la pianta
K_xi = K_z(m+1:end);

% Costruzione del compensatore H(s)
% H(s) = (k_{m-1}*s^{m-1} + ... + k_1*s + k_0) / (s^m + alpha_{m-1}*s^{m-1} + ... + alpha_0)
numH = fliplr(k_H);  % flip per avere coefficienti in ordine decrescente di s
denH = alpha;        % coefficienti del polinomio caratteristico


%% FILTRO DERIVATIVO PER LA STIMA DELLA VELOCITÀ

% Parametri del filtro derivativo del secondo ordine
omega_filter = 10*omega_n;  % Frequenza naturale del filtro
delta_filter = 1/sqrt(2); % Smorzamento del filtro

% Funzione di trasferimento del filtro derivativo
% F(s) = (omega_f^2 * s) / (s^2 + 2*delta_f*omega_f*s + omega_f^2)
num_filter = [omega_filter^2, 0];
den_filter = [1, 2*delta_filter*omega_filter, omega_filter^2];
