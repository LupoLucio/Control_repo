% TEMPI DI CAMPIONAMENTO DA TESTARE
T1 = 0.001;   % 1 ms
T2 = 0.010;   % 10 ms
T3 = 0.050;   % 50 ms

% Scegli quello da usare in questo momento:
T = T2;

z = tf('z', T);

s_lap_euler_forward = (z-1)/ (T);

C_z_euler_forward = kp + ki/s_lap_euler_forward + kd*s_lap_euler_forward/(1 + tl*s_lap_euler_forward);

C_z_euler_forward_real = minreal(C_z_euler_forward);