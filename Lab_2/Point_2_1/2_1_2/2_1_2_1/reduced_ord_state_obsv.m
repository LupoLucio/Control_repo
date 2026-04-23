%% 1. COSTRUZIONE MATRICI A, B, C (Modello continuo)
% State-space matrices of gear motor
A = [ 0      1;

     0      -(Req*Beq+mot.Kt*mot.Ke)/(Req*Jeq)];

B = [0; (drv.dcgain*mot.Kt)/(gbox.N*Req*Jeq)];

C = [1 0];

D = 0;

% specs 2 order system
wn = wgc;

sigma = -delta*wn;
wd = wn*sqrt(1-delta^2);

p1 = sigma + 1j * wd;
p2 = sigma - 1j * wd;

poles_des = [p1 p2];

% calcolo K 
K = place(A, B, poles_des);

% calcolo Nu e Nx
M = [A B; C D];
rhs = [0;0;1];
sol = M\rhs;

Nx = sol(1:2);% vector
Nu = sol(3);% scalar
Nr = Nu + K*Nx;


%% 2. POLI DEL CONTROLLORE STATE-FEEDBACK

p_ctrl = [-delta*wn + 1j*wn*sqrt(1-delta^2), ...
          -delta*wn - 1j*wn*sqrt(1-delta^2)];

% Calcolo guadagno K (solo per riferimento, non strettamente necessario per l'osservatore)
K = place(A, B, p_ctrl);

% Parte reale dominante dei poli del controllore (decadimento)
sigma_c = abs(real(p_ctrl(1)));

%% 3. PROGETTAZIONE OSSERVATORE RIDOTTO (Punto 2.1.2.1)
% Il modello è già partizionato: x1=θ (misurato), x2=ω (da stimare)
% Quindi T = I e le sottomatrici sono immediate:
A11 = 0; A12 = 1;
A21 = 0; A22 = A(2,2);
B1  = 0; B2  = B(2);

% Requisito: polo osservatore 5 volte più veloce del controllore
lambda_o = -5 * sigma_c;  % Negativo per stabilità

% Calcolo guadagno L dall'equazione Ao = A22 - L*A12
L = A22 - lambda_o;    
% (Nota: A22 è negativo, quindi L risulterà positivo)

fprintf('\n=== Progetto Osservatore Ridotto ===\n');
fprintf('Polo osservatore λo = %.2f rad/s\n', lambda_o);
fprintf('Guadagno osservatore L  = %.4f\n', L);

%% 4. CALCOLO MATRICI OSSERVATORE (Eq. 8 dell'Assignment)
Ao = A22 - L*A12;
Bo = [B2 - L*B1,  (A22 - L*A12)*L + A21 - L*A11];
Co = [0; 1];
Do = [0, 1; 0, L];

%% 5. VERIFICA NUMERICA
fprintf('\n=== Verifica ===\n');
fprintf('Autovalore calcolato di Ao: %.2f (deve essere %.2f)\n', eig(Ao), lambda_o);
fprintf('Dimensioni: Ao(1x1), Bo(1x2), Co(2x1), Do(2x2) ✓\n');

%% 6 Matrici oservatore discreto

Phi_o = 1 + Ao*T;              
Gam_o = Bo*T;
H_o   = Co;
J_o   = Do;