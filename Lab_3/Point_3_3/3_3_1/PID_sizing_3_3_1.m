%% Simplified Motor transfer function
km = drv.dcgain / mot.Ke;   
Req = mot.R + sens.curr.Rs;
Jeq = mot.J + mld.Jh / (gbox.N^2);
Beq = mot.B + mld.Bh / (gbox.N^2); 
Tm = (Req * Jeq) / (mot.Kt * mot.Ke); 
s = tf('s');
term_1 = drv.dcgain/(1+drv.Tc*s);
term_2 = 1/(gbox.N*s);
num_3 = mot.Kt*((mld.Jb*s^2)+(mld.Bb*s)+mld.k);
a1 = Jeq*mld.Jb;
a2 = Jeq*mld.Bb+mld.Jb*Beq;
a3 = Beq*mld.Bb+mld.k*(Jeq+((mld.Jb)/((gbox.N)^2)));
a4 = mld.k*(Beq+((mld.Bb)/((gbox.N)^2)));
D_tau_prime = a1*s^3+a2*s^2+a3*s+a4;
den_3 = D_tau_prime*(mot.L*s+Req)+mot.Kt*mot.Ke*((mld.Jb*s^2)+(mld.Bb*s)+mld.k);
term_3 = num_3/den_3;
P_u_theta_h = term_1*term_2*term_3;
[num_P_u_theta_h, den_P_u_theta_h] = tfdata(P_u_theta_h, 'v'); 

%% PID required parameters
ts_5 = 0.85;
K_W = 5/ts_5;
Mp = 0.3;            
alpha = 4;

signal_type = "linear ramp";

[kp, ki, kd, tl, type] = controller_design_lab_3(Mp,ts_5,P_u_theta_h,alpha,signal_type);

s = tf('s');

C = kp + ki/s + kd*s/(1 + tl*s);

%% Open loop TF
L = C * P_u_theta_h;
H = L/(1+L);

[GM, PM, Wcg, Wcp] = margin(L);

fprintf('PIDF sintetizzato:\n');
fprintf('  PM ottenuto = %.2f°\n', PM);
fprintf('  wc ottenuta = %.2f rad/s\n', Wcp);

margin(L)
grid on

%% Requirements verification
out = pid_metrics(P_u_theta_h,kp,ki,kd,tl);