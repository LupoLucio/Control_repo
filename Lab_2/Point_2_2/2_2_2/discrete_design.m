T = 0.001;
% 1. Modello discreto (già fatto al punto 1)
[Phi, Gam, H, J] = ssdata(c2d(ss(A,B,C,D), T, 'zoh'));

% 2. Mappatura poli continuo -> discreto
s_ctrl = [-delta*wn + 1j*wn*sqrt(1-delta^2), -delta*wn - 1j*wn*sqrt(1-delta^2)];
z_ctrl = exp(s_ctrl * T);
lambda_o_cont = -5 * abs(real(s_ctrl(1)));
z_obs = exp(lambda_o_cont * T);

% 3. Feedback K (pole placement discreto)
K = place(Phi, Gam, z_ctrl);

% 4. Feedforward Nx, Nu (Eq. 19)
M_feed = [Phi - eye(2), Gam; H, 0];
rhs = [0; 0; 1];
sol = M_feed \ rhs;
Nx = sol(1:2); Nu = sol(3);
Nr = Nu + K*Nx;

% 5. Osservatore ridotto discreto (Eq. 14)
Phi11 = Phi(1,1); Phi12 = Phi(1,2);
Phi21 = Phi(2,1); Phi22 = Phi(2,2);
Gam1  = Gam(1);   Gam2  = Gam(2);

L = (Phi22 - z_obs) / Phi12;          % Guadagno osservatore
Phi_o = Phi22 - L*Phi12;              % = z_obs (verifica)
Gam_o = [Gam2 - L*Gam1,  Phi_o*L + Phi21 - L*Phi11];
H_o   = [0; 1];
J_o   = [0, 1; 0, L];