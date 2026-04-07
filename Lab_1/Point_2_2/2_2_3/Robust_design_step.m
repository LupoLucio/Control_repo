%% --- MODELLI DI BASE (A, B, C) ---
% Qui assumi che A, B, C siano già definiti dal modello SRV02
% A = [...]; B = [...]; C = [1 0];

%% --- SISTEMA ESTESO ---
A_e = [ 0    C;
        zeros(2,1)  A ];

B_e = [0;
       B];

C_e = [0  C];
D_e = 0;


% state space model
sys_e = ss(A_e,B_e,C_e,D_e);
G_e = tf(sys);
[num_G_e,den_G_e] = tfdata(G_e, 'v');

%% --- CONTROLLABILITÀ ---
disp("Rank controllabilità A_e,B_e = " + rank(ctrb(A_e,B_e)));

%% --- SCELTA DEI POLI AMMISSIBILI ---
lambda_placements = [
    sigma + 1j*wd,   sigma - 1j*wd,   sigma;      % caso 1
    2*sigma + 1j*wd, 2*sigma - 1j*wd, 2*sigma;    % caso 2
    2*sigma + 1j*wd, 2*sigma - 1j*wd, 3*sigma     % caso 3
];

%% --- CALCOLO DEI GUADAGNI K ---
K_vec = zeros(3,3);

for i = 1:3
    poles_des = lambda_placements(i,:);
    K_vec(i,:) = place(A_e, B_e, poles_des);
end

%% --- CALCOLO Nx e Nu PER OGNI SET DI POLI ---
Nx_vec = zeros(3,3);
Nu_vec = zeros(3,1);

M = [A_e B_e; C_e D_e];
rhs = [0;0;0;1];

for i = 1:3
    sol = M\rhs;
    Nx_vec(i,:) = sol(1:3).';
    Nu_vec(i)   = sol(4);
end

%% --- OUTPUT ---
disp("Guadagni K per ciascun caso:")
disp(K_vec)

disp("Nx per ciascun caso:")
disp(Nx_vec)

disp("Nu per ciascun caso:")
disp(Nu_vec)
