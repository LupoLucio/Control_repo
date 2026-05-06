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

disp('A_prime ='); disp(A_prime);
disp('B_prime ='); disp(B_prime);

%% ============================================================
% 2. Specifiche → poli desiderati (eq. 8-10)
% ============================================================

ts  = 0.65;   % Settling time richiesto
Mp  = 0.25;   % Overshoot max

delta_val = log(1/Mp) / sqrt(pi^2 + log(1/Mp)^2);
wn        = 3 / (delta_val * ts);

% Coppia dominante
sigma = -delta_val * wn;
wd    =  wn * sqrt(1 - delta_val^2);

p1 = sigma + 1i*wd;
p2 = sigma - 1i*wd;

% Coppia secondaria (più veloce)
p3 = 2*sigma + 1i*(2*wd);
p4 = 2*sigma - 1i*(2*wd);

desired_poles = [p1 p2 p3 p4];

disp('Poli desiderati ='); disp(desired_poles.');

%% ============================================================
% 3. Calcolo guadagno K
% ============================================================

K = place(A_prime, B_prime, desired_poles);
K = real(K);

disp('K ='); disp(K);

eig_cl = eig(A_prime - B_prime*K);
disp('Autovalori A-BK ='); disp(eig_cl);

%% ============================================================
% 4. Feedforward corretto (tracking perfetto)
% ============================================================

A_cl = A_prime - B_prime*K;

Kff = -1 / (C * (A_cl \ B_prime));

disp(['Kff = ', num2str(Kff)]);

%% ============================================================
% 5. Simulazione risposta al gradino (2.3.2)
% ============================================================

sys_cl_ff = ss(A_cl, B_prime*Kff, C, D);

t_sim  = linspace(0, 2, 2000);
ref_deg = 50;
ref_rad = deg2rad*ref_deg;

[y, t] = step(ref_rad * sys_cl_ff, t_sim);

figure;
plot(t, y * rad2deg, 'b', 'LineWidth', 1.5); hold on;
yline(ref_deg,        'r--', 'Riferimento', 'LineWidth', 1.2);
yline(ref_deg * 1.05, 'k:',  '+5%');
yline(ref_deg * 0.95, 'k:',  '-5%');
grid on;
title('Risposta al gradino - State-Space (Eigenvalue Placement)');
xlabel('Tempo [s]');
ylabel('\theta_h [deg]');
legend('Risposta', 'Riferimento');

info = stepinfo(y * rad2deg, t, ref_deg);

fprintf('\n=== Metriche risposta ===\n');
fprintf('Settling Time (5%%): %.3f s (<= %.2f)\n', info.SettlingTime, ts);
fprintf('Overshoot: %.2f %% (<= %.0f %%)\n', info.Overshoot, Mp*100);
fprintf('Peak Time: %.3f s\n', info.PeakTime);
