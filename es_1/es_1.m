
% -------------------------------------------------------------------------
% MODEL 
% -------------------------------------------------------------------------

% mass
m1 = 10; m2 = 10; m3 = 10;

% spring coeff
k1 = 10; k2 = 10;

% dumpers coeff
c1 = 0.3; c2 = 0.3;

% friction coeff
x1 = 0; x2 = 0; x3 = 0;

% System matrix
A = [ 0      0        0        1        0        0;
      0      0        0        0        1        0;
      0      0        0        0        0        1;
      -k1/m1 k1/m1    0   -(c1+x1)/m1  c1/m1     0;
      k1/m2  -(k1+k2)/m2   k2/m2   c1/m2    -(c1+c2+x2)/m2  c2/m2;
      0      k2/m3        -k2/m3    0   c2/m3 -(c2+x3)/m3 ];

% Input matrix
B = [0; 0; 0; 0; 0; 1/m3];

% Output matrix
C = [1 0 0 0 0 0];

% Feedthrough matrix
D = 0;

sys = ss(A,B,C,D);

% -------------------------------------------------------------------------
% 1 Check the stability of the system (simple/asymptotic/BIBO) 
%  and plot poles and zeros of the system.
% -------------------------------------------------------------------------



eigenval = eig(A);

poles = rlocus(sys);

% no eigenvalues with real part > 0
% simple stability, asymptotic stability, bibo stability


% -------------------------------------------------------------------------
% 2 Plot the impulse response of the system.
%  Is the response bounded?
% -------------------------------------------------------------------------

figure;
impulse(sys, 1000);
title('Impulse response (no friction)');
grid on;

% yes

% -------------------------------------------------------------------------
% 3 Plot the free response of both 
% the output (y(t)) and the state (x(t)) 
% to the initial condition x0 = [1, 0, 0, 0, 0, 0]T.
% Are the state and output signals bounded? Why is that?
% -------------------------------------------------------------------------

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

% state and output signals are bounded but oscillates


% -------------------------------------------------------------------------
% 4 Plot the free response of both
% the output (y(t)) and the state (x(t))
% to the initial condition x0 = [0, 0, 0, 1, 0, 0]T.
% Are the state and output signals bounded?
% -------------------------------------------------------------------------

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

% state and output signals are not bounded since all the vagons moves
% and there is no friction.









