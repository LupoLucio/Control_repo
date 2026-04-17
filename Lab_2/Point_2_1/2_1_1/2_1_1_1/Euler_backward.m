% TEMPI DI CAMPIONAMENTO DA TESTARE
T1 = 0.001;   % 1 ms
T2 = 0.010;   % 10 ms
T3 = 0.050;   % 50 ms

% Scegli quello da usare in questo momento:
T = T2;

z = tf('z', T);

s_lap_euler_backward = (z-1)/ (z*T);


C_z_euler_backward = kp + ki/s_lap_euler_backward + kd*s_lap_euler_backward/(1 + tl*s_lap_euler_backward);

C_z_euler_backward_real = minreal(C_z_euler_backward);


