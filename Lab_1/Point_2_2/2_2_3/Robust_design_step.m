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
G_e = tf(sys_e);
[num_G_e,den_G_e] = tfdata(G_e, 'v');

%% --- CONTROLLABILITÀ ---
disp("Rank controllabilità A_e,B_e = " + rank(ctrb(A_e,B_e)));

%% --- SCELTA DEI POLI AMMISSIBILI ---
lambda_placements = [
    sigma + 1j*wd,   sigma - 1j*wd,   sigma;      % caso 1
    sigma, sigma, sigma                           % caso 2
    2*sigma + 1j*wd, 2*sigma - 1j*wd, 2*sigma;    % caso 3
    2*sigma + 1j*wd, 2*sigma - 1j*wd, 3*sigma     % caso 4
];

%% --- CALCOLO DEI GUADAGNI K ---
K_vec = zeros(3,3);

for i = 1:4
    poles_des = lambda_placements(i,:);
    if (i==2)
        K_vec(i,:) = acker(A_e, B_e, poles_des);
    else
        K_vec(i,:) = place(A_e, B_e, poles_des);
    end
end

index = 2;

%% --- OUTPUT ---
disp("Guadagni K per ciascun caso:")
disp(K_vec)
