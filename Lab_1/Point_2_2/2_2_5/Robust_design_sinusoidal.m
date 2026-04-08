n = size(A,1);

%% =========================
% 2. MODELLO INTERNO (alpha)
% =========================
% Esempio: tracking di gradino → m = 1 → s = 0
alpha = [1 0];   % s

m = length(alpha) - 1;

%% =========================
% 3. COSTRUZIONE SISTEMA ESTESO (Az, Bz)
% =========================

% Dimensione totale: m + n
Az = zeros(m+n);
Bz = zeros(m+n,1);

% Parte errore (catena integratori)
for i = 1:m-1
    Az(i, i+1) = 1;
end

% Riga dinamica errore
Az(m,1:m) = -fliplr(alpha(2:end));  % [-alpha0 ... -alpha_m-1]
Az(m,m+1:end) = C;

% Dinamica ξ
Az(m+1:end, m+1:end) = A;

% Ingresso
Bz(end-n+1:end) = B;

%% =========================
% 4. PROGETTO Kz
% =========================
% Poli desiderati (scegli tu)
poles = [-2 -3 -4];

Kz = place(Az, Bz, poles);

% Separazione guadagni
k = Kz(1:m);           % k0 ... k_m-1
Kxi = Kz(m+1:end);     % Kξ

%% =========================
% 5. COSTRUZIONE H(s)
% =========================

% Numeratore: k_m-1 ... k0
num = fliplr(k);

% Denominatore: s^m + ...
den = alpha;

H = tf(num, den);

%% =========================
% 6. OUTPUT
% =========================
disp('Kxi = ');
disp(Kxi);

disp('H(s) = ');
H

%% =========================
% 7. VERIFICA (opzionale)
% =========================
Acl = Az - Bz*Kz;
disp('Poli sistema chiuso:');
disp(eig(Acl));