% TEMPI DI CAMPIONAMENTO DA TESTARE
T1 = 0.001;   % 1 ms
T2 = 0.010;   % 10 ms
T3 = 0.050;   % 50 ms

% Scegli quello da usare in questo momento:
T = T2;

z = tf('z', T);

s_lap = (z-1)/ (z*T);


C_z = kp + ki/s_lap + kd*s_lap/(1 + tl*s_lap);

% Risultato da confrontare
C_P = kp;
C_I = ki * (T*z) / (z - 1);
C_D = kd * (z - 1) / ((tl + T)*z - tl);

% Controllore completo
C_pid_z = (C_P + C_I + C_D);



C_z_real = minreal(C_z);
C_pid_z_real = minreal(C_pid_z);


