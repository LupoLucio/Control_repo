% TEMPI DI CAMPIONAMENTO DA TESTARE
T1 = 0.001;   % 1 ms
T2 = 0.010;   % 10 ms
T3 = 0.050;   % 50 ms

% Scegli quello da usare in questo momento:
T = T2;

z = tf('z', T);

s_lap_tustin =(2/T)*((z-1)/(z-2));

C_z_tustin = kp + ki/s_lap + kd*s_lap/(1 + tl*s_lap);

C_z_tustin_real = minreal(C_z_tustin);
