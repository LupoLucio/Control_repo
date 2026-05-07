%% 1. Specifiche e Geometria Regione Ammissibile
ts_max = 0.85;  % [s]
Mp_max = 0.30;  % [adim]

delta_val = log(1/Mp_max) / sqrt(pi^2 + log(1/Mp_max)^2);
wn_min    = 3 / (delta_val * ts_max);
phi       = atan(sqrt(1 - delta_val^2) / delta_val);
sigma_lim = -3 / ts_max;  % Re(λ) <= -3.529

fprintf('📐 Regione Ammissibile:\n');
fprintf('   Re(λ) <= %.3f rad/s\n', sigma_lim);
fprintf('   Smorzamento δ >= %.3f  (φ = %.2f°)\n\n', delta_val, rad2deg*phi);

%% 2. Modello State-Space (Assicurati che le variabili siano nel workspace)
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

sysG  = ss(A_prime, B_prime, C, D);
sysGp = ss(-A_prime, -B_prime, C, D); % G(-s)
sysL  = sysGp * sysG;     % Loop per SRL

%% 3. Plot SRL + Regione Ammissibile
figure('Name','SRL - Scelta peso r','Color','w','Position',[100 100 800 600]);
rlocus(sysL); hold on; grid on;

% Disegno settore ammissibile
r_max = 60;
x_sec = [sigma_lim, -r_max, -r_max, sigma_lim];
y_sec = [r_max*tan(phi), r_max*tan(phi), -r_max*tan(phi), -r_max*tan(phi)];
fill(x_sec, y_sec, 'g', 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'DisplayName','Regione Ammissibile');
xline(sigma_lim, 'r--', 'LineWidth',1.5, 'DisplayName','Limite ts (σ)');
plot([0 sigma_lim], [0 sigma_lim*tan(phi)], 'r:', 'LineWidth',1.5, 'DisplayName','Limite Mp (+φ)');
plot([0 sigma_lim], [0 -sigma_lim*tan(phi)], 'r:', 'LineWidth',1.5, 'DisplayName','Limite Mp (-φ)');
xlim([-60 10]); ylim([-60 60]);
title('Symmetric Root Locus: Scelta di r (guadagno plot k = 1/r)');
xlabel('Real Axis'); ylabel('Imaginary Axis');
legend('Location','Best');

%% 4. Scelta di r (Opzione A: Interattiva con rlocfind)
fprintf('💡 CLICCA con il mouse all''interno della zona verde per scegliere k = 1/r...\n');
[k_chosen, ~] = rlocfind(sysL); % Attende il click
r_chosen = 1/k_chosen;
fprintf('✅ k scelto = %.3e  →  r = %.3e\n\n', k_chosen, r_chosen);

%% 5. Calcolo LQR e Verifica Specifiche
% lqry minimizza ∫(y^2 + r u^2) dt → corrisponde a J = ∫(ϑh^2 + r u^2) dt
K = lqry(sysG, 1, r_chosen);

% Verifica poli anello chiuso
sys_cl = ss(A_prime - B_prime*K, B_prime, C, D);
p_cl   = eig(sys_cl);
re_p   = real(p_cl);
im_p   = imag(p_cl);
mag_p  = abs(p_cl);
zeta_p = -re_p ./ mag_p; % Smorzamento effettivo

fprintf('🔍 Verifica Poli Anello Chiuso:\n');
for i=1:length(p_cl)
    fprintf('   λ%d = %+.3f %+.3fj  |  Re=%+.3f  |  ζ=%.3f\n', ...
        i, re_p(i), im_p(i), re_p(i), zeta_p(i));
end

viol_ts = any(re_p > sigma_lim);
viol_Mp = any(zeta_p < delta_val & abs(im_p) > 0.1); % Ignora poli reali puri

if ~viol_ts && ~viol_Mp
    fprintf('✅ TUTTE LE SPECIFICHE SONO SODDISFATTE.\n');
else
    fprintf('⚠️  Specifiche NON soddisfatte. Riprova con un k più alto (r più basso).\n');
end

%% 6. Feedforward per Tracking Nominale
Kff = 1 / dcgain(sys_cl);
fprintf('\n📦 Guadagni da implementare:\n');
fprintf('   K   = [%.4f, %.4f, %.4f, %.4f]\n', K);
fprintf('   Kff = %.4f\n', Kff);

% Salvataggio workspace
assignin('base','K_lqr', K);
assignin('base','Kff_lqr', Kff);