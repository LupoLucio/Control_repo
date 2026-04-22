reduced_ord_state_obsv;

%% 1. Definizione Sampling Time (Parametrico)
T = 0.010; % Cambia in T1=0.001 o T3=0.050 per i test successivi

%% 2. Discretizzazione Esatta (ZOH) della Pianta
% Assumiamo che A, B, C, D siano già definiti nel workspace dal Lab 1
sys_c = ss(A, B, C, D);          % Modello continuo
sys_d = c2d(sys_c, T, 'zoh');    % Discretizzazione esatta (ZOH)

%% 3. Estrazione Matrici Discrete (Φ, Γ, H, J)
[Phi, Gam, H, J] = ssdata(sys_d);

%% 4. Verifica & Output
fprintf('=== Discretizzazione ZOH Pianta (T = %.0f ms) ===\n', T*1000);
fprintf('Phi = \n'); disp(Phi);
fprintf('Gamma = \n'); disp(Gam);
fprintf('H = \n'); disp(H);
fprintf('J = %.2f (Deve essere 0 per D=0) \n', J);

% Salvataggio automatico per i punti successivi
save('PiantaDiscreta_2_2.mat', 'Phi', 'Gam', 'H', 'J', 'T');