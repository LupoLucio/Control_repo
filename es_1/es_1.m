
% 1 sistema

% Parameters
m1 = 10; m2 = 10; m3 = 10;
k1 = 10; k2 = 10;
c1 = 0.3; c2 = 0.3;

% No viscous friction
x1 = 0; x2 = 0; x3 = 0;

% State-space matrices
A = [ 0      0        0        1        0        0;
      0      0        0        0        1        0;
      0      0        0        0        0        1;
      -k1/m1 k1/m1    0   -(c1+x1)/m1  c1/m1     0;
      k1/m2  -(k1+k2)/m2   k2/m2   c1/m2    -(c1+c2+x2)/m2  c2/m2;
      0      k2/m3        -k2/m3    0   c2/m3 -(c2+x3)/m3 ];

B = [0; 0; 0; 0; 0; 1/m3];

C = [1 0 0 0 0 0];   % GPS on first wagon



sys = ss(A,B,C,0);


% 1 poli , stabilità

figure;
pzmap(sys);
grid on;

figure; 
rlocus(sys); 
grid on; 
title('Root Locus sistema con attrito');

% no poles with real part > 0
% simple stability, no bibo no asymptotic


% 2 impulse response
figure;
impulse(sys, 1000);
title('Impulse response (no friction)');
grid on;

% no BIBO stable 

% 3 free responde 

x0 = [1 0 0 0 0 0]';
t = 0:1:1000;

[y1, t1, x1] = initial(sys, x0, t);

figure;
plot(t1, y1);
title('Free response y(t) from x0 = [1 0 0 0 0 0]');
grid on;

figure;
plot(t1, x1);
title('State response x(t) from x0 = [1 0 0 0 0 0]');
grid on;

% signals are bounded but oscillates
% the inizial energy remains constant in the system
% 

% 4 free response

x0 = [0 0 0 1 0 0]';
[y2, t2, x2] = initial(sys, x0, t);

figure;
plot(t2, y2);
title('Free response y(t) from x0 = [0 0 0 1 0 0]');
grid on;

figure;
plot(t2, x2);
title('State response x(t) from x0 = [0 0 0 1 0 0]');
grid on;

% signals y(t) not bounded cause all the trains moves
% cause there is no friction









