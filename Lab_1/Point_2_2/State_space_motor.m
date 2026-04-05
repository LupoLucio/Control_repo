

% State-space matrices of gear motor
A = [ 0      1;

     0      -(Req*Beq+mot.Kt*mot.Ke)/(Req*Jeq)];

B = [0; (drv.dcgain*mot.Kt)/(gbox.N*Req*Jeq)];

C = [1 0];

D = 0;

% specs 2 order system
delta = 0.6;
wn = 45;

p1 = -delta*wn + 1j*wn*sqrt(1-delta^2);
p2 = -delta*wn - 1j*wn*sqrt(1-delta^2);

poles_des = [p1 p2];

% calcolo K 
K = place(A, B, poles_des);

% calcolo Nu e Nx
M = [A B; C 0];
rhs = [0;0;1];
sol = M\rhs;

Nx = sol(1:2);% vector
Nu = sol(3);% scalar

Nr = Nu + K*Nx;

% controllore finale
%u = -K*x + Nr*r


% check
rank(ctrb(A,B))