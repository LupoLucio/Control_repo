% TEMPI DI CAMPIONAMENTO DA TESTARE
T1 = 0.001;   % 1 ms
T2 = 0.010;   % 10 ms
T3 = 0.050;   % 50 ms

% Scegli quello da usare in questo momento:
T = T2;

% 3. Discretizzazione con Eulero all'indietro (metodo spazio di stato)
% Estrazione matrici di stato
[A, B, C, D] = ssdata(C_s);
n = size(A, 1);
I = eye(n);

% Formule di discretizzazione Eulero all'indietro (implicito)
% x[k] = x[k-1] + Ts*(A*x[k] + B*u[k])  =>  (I - Ts*A)*x[k] = x[k-1] + Ts*B*u[k]
Ad = inv(I - T*A);
Bd = T * Ad * B;
Cd = C * Ad;
Dd = D + T * C * Ad * B;

% Creazione sistema discreto
C_z_ss = ss(Ad, Bd, Cd, Dd, T);
C_z = tf(C_z_ss); % Conversione in TF discreta





z = tf('z', T);

% Tre termini separati
C_P = kp;
C_I = ki * (T*z) / (z - 1);
C_D = kd * (z - 1) / ((tl + T)*z - tl);

% Controllore completo
C_pid_z = C_P + C_I + C_D;


