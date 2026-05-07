A_prime = zeros(4);

A_prime(1,3) = 1;
A_prime(2,4) = 1;
A_prime(3,2) =  mld.k/(gbox.N^2 * Jeq);
A_prime(3,3) = -(1/Jeq) * (Beq + (mot.Kt*mot.Ke)/Req);
A_prime(4,2) = -(mld.k/mld.Jb)-(mld.k/(Jeq*gbox.N^2));
A_prime(4,3) = -(mld.Bb/mld.Jb)+(1/Jeq) * (Beq + (mot.Kt*mot.Ke)/Req);
A_prime(4,4) = -mld.Bb/mld.Jb;

B_prime = [0;
           0;
           (mot.Kt*drv.dcgain)/(gbox.N*Jeq*Req);
          -(mot.Kt*drv.dcgain)/(gbox.N*Jeq*Req)];

C = [1 0 0 0];
D = 0;

%% 2.4.3 - LQR con Regola di Bryson

fprintf('\n========================================\n');
fprintf('2.4.3 - LQR CON REGOLA DI BRYSON\n');
fprintf('========================================\n');

%% 1. Specifiche e Calcolo Deviazioni Massime
theta_h_star = 50 * deg2rad;  % Set-point di 50 gradi in rad

% Deviazioni massime ammissibili (dalle eq. 15 dell'Assignment)
theta_h_bar = 0.3 * 50 * deg2rad;  % 0.3 * 50° = 15° = 0.2618 rad
theta_d_bar = pi/36;                % 5° = 0.0873 rad
u_bar = 10;                         % 10 V

fprintf('\n📐 Deviazioni Massime Ammissibili:\n');
fprintf('   |ϑh - ϑh*| < %.4f rad (%.1f deg)\n', theta_h_bar, theta_h_bar*rad2deg);
fprintf('   |ϑd|      < %.4f rad (%.1f deg)\n', theta_d_bar, theta_d_bar*rad2deg);
fprintf('   |u|       < %.1f V\n', u_bar);

%% 2. Calcolo Pesi con Regola di Bryson
% Bryson's rule: q_ii = 1/(x_i_bar)^2, r = 1/(u_bar)^2
q11 = 1 / (theta_h_bar^2);  % Peso per ϑh
q22 = 1 / (theta_d_bar^2);  % Peso per ϑd
q33 = 0;                     % Peso per ϑ˙h (zero come richiesto)
q44 = 0;                     % Peso per ϑ˙d (zero come richiesto)
r_bryson = 1 / (u_bar^2);    % Peso per l'input u

fprintf('\n⚖️ Pesi di Bryson:\n');
fprintf('   q11 (ϑh)  = %.4f\n', q11);
fprintf('   q22 (ϑd)  = %.4f\n', q22);
fprintf('   q33 (ϑ˙h) = %.4f\n', q33);
fprintf('   q44 (ϑ˙d) = %.4f\n', q44);
fprintf('   r (u)     = %.4f\n', r_bryson);

% Matrice Q diagonale
Q_bryson = diag([q11, q22, q33, q44]);

fprintf('\n📋 Matrice Q:\n');
disp(Q_bryson);

%% 3. Calcolo LQR
% Uso lqr(sysP, Q, R) dove sysP è il modello della pianta
K_bryson = lqr(A_prime, B_prime, Q_bryson, r_bryson);

fprintf('✅ Matrice di guadagno K calcolata:\n');
fprintf('   K = [%.4f, %.4f, %.4f, %.4f]\n', K_bryson);

%% 4. Verifica Autovalori Anello Chiuso
sys_cl_bryson = ss(A_prime - B_prime*K_bryson, B_prime, C, D);
p_cl_bryson = eig(sys_cl_bryson);

fprintf('\n🔍 Autovalori Anello Chiuso:\n');
for i = 1:length(p_cl_bryson)
    lambda = p_cl_bryson(i);
    wn_i = abs(lambda);
    if abs(lambda) > 1e-10
        zeta_i = -real(lambda) / wn_i;
    else
        zeta_i = 0;
    end
    fprintf('   λ%d = %8.4f %+.4fj  |  ωn=%.4f rad/s  |  ζ=%.4f\n', ...
        i, real(lambda), imag(lambda), wn_i, zeta_i);
end

%% 5. Verifica Specifiche
% Calcolo parametri dai poli dominanti
[poli_ordinati, idx] = sort(abs(real(p_cl_bryson)));
poli_dominanti = p_cl_bryson(idx(end-1:end));  % I due poli più lenti

% Stima parametri secondo ordine approssimato
if imag(poli_dominanti(1)) ~= 0
    % Poli complessi coniugati
    sigma_dom = -real(poli_dominanti(1));
    omega_d = abs(imag(poli_dominanti(1)));
    wn_dom = abs(poli_dominanti(1));
    zeta_dom = sigma_dom / wn_dom;
else
    % Poli reali
    sigma_dom = -min(real(poli_dominanti));
    wn_dom = sigma_dom;
    zeta_dom = 1;
end

ts_5pct = 3 / sigma_dom;
Mp = exp(-pi * zeta_dom / sqrt(1 - zeta_dom^2)) * 100;

fprintf('\n📊 Prestazioni Stimate (dai poli dominanti):\n');
fprintf('   ωn (dom) = %.4f rad/s\n', wn_dom);
fprintf('   ζ (dom)  = %.4f\n', zeta_dom);
fprintf('   ts,5%%    = %.4f s (spec: ≤ 0.85 s) → %s\n', ...
    ts_5pct, ternary(ts_5pct <= 0.85, '✅', '❌'));
fprintf('   Mp       = %.2f%% (spec: ≤ 30%%) → %s\n', ...
    Mp, ternary(Mp <= 30, '✅', '❌'));

%% 6. Calcolo Feedforward per Tracking Nominale
Kff_bryson = 1 / dcgain(sys_cl_bryson);
fprintf('\n📦 Guadagno Feedforward:\n');
fprintf('   Kff = %.4f\n', Kff_bryson);

%% 7. Salvataggio nel Workspace
assignin('base', 'K_bryson', K_bryson);
assignin('base', 'Kff_bryson', Kff_bryson);
assignin('base', 'Q_bryson', Q_bryson);
assignin('base', 'r_bryson', r_bryson);

fprintf('\n💾 Variabili salvate nel workspace:\n');
fprintf('   K_bryson, Kff_bryson, Q_bryson, r_bryson\n');

%% Funzione helper per ternary operator
function result = ternary(condition, valTrue, valFalse)
    if condition
        result = valTrue;
    else
        result = valFalse;
    end
end