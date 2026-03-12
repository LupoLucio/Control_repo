% 5 

% Parameters
m1 = 10; m2 = 10; m3 = 10;
k1 = 10; k2 = 10;
c1 = 0.3; c2 = 0.3;

% friction
x1 = 0.2; x2 = 0.2; x3 = 0.2;

% State-space matrices
A = [ 0      0        0        1        0        0;
      0      0        0        0        1        0;
      0      0        0        0        0        1;
      -k1/m1 k1/m1    0   -(c1+x1)/m1  c1/m1     0;
      k1/m2  -(k1+k2)/m2   k2/m2   c1/m2    -(c1+c2+x2)/m2  c2/m2;
      0      k2/m3        -k2/m3    0   c2/m3 -(c2+x3)/m3 ];

B = [0; 0; 0; 0; 0; 1/m3];

C = [1 0 0 0 0 0];   % GPS on first wagon


sys_f = ss(A,B,C,0);

%1
figure;
pzmap(sys_f); 
grid on;


%2
figure;
impulse(sys_f,200);
title('Impulse response with friction');
grid on;

%3
x0 = [1 0 0 0 0 0]';
t = 0:0.01:20;
[y1,t1,x1] = initial(sys_f,x0,t);

figure; plot(t1,y1); title('y(t) with friction'); grid on;
figure; plot(t1,x1); title('x(t) with friction'); grid on;

%4
x0 = [0 0 0 1 0 0]';
t = 0:0.01:20;
[y1,t1,x1] = initial(sys_f,x0,t);

figure; plot(t1,y1); title('y(t) with friction'); grid on;
figure; plot(t1,x1); title('x(t) with friction'); grid on;


% 6
[mag,phase,w] = bode(sys_f);
mag_db = 20*log10(squeeze(mag));

idx1 = find(mag_db <= -28, 1, 'first');
idx2 = find(mag_db <= -50, 1, 'first');

w1 = w(idx1);
w2 = w(idx2);

f1 = w1/(2*pi)
f2 = w2/(2*pi)

Ts1 = 1/f1;
Ts2 = 1/f2;

sysd1 = c2d(sys_f, Ts1, 'zoh');
sysd2 = c2d(sys_f, Ts2, 'zoh');

figure;
subplot(3,1,1); initial(sys_f, x0, 20); title('Continuous');
subplot(3,1,2); initial(sysd1, x0, 20); title('Discrete f1');
subplot(3,1,3); initial(sysd2, x0, 20); title('Discrete f2');

% 7

figure; 
rlocus(sys_f); 
grid on; 
title('Root Locus sistema con attrito');

Kvec = linspace(0,5000,50000);   % tanti valori di K
poles = rlocus(sys_f, Kvec);

% Trova il primo K per cui un polo entra nel semipiano destro
stable = true(size(Kvec));

for i = 1:length(Kvec)
    if any(real(poles(:,i)) >= 0)
        stable(i) = false;
    end
end

% Il massimo K stabile è l'ultimo K prima dell'instabilità
Kmax = Kvec(find(stable,1,'last'))

% 8 

Ob = obsv(A,C);
rank_Ob = rank(Ob)

% 9

desired_poles = [-1 -2 -3 -4 -5 -6];
K = acker(A,B,desired_poles)

