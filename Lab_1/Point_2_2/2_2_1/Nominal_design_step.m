% State-space matrices of gear motor
A = [ 0      1;

     0      -(Req*Beq+mot.Kt*mot.Ke)/(Req*Jeq)];

B = [0; (drv.dcgain*mot.Kt)/(gbox.N*Req*Jeq)];

C = [1 0];

D = 0;

% state space model
sys = ss(A,B,C,D);
G = tf(sys);
[num_G,den_G] = tfdata(G, 'v');


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


