% 9 

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

C = [1 -1 0 0 0 0];


%10

%Check of the observability
Ob = obsv(A,C);
rank_Ob = rank(Ob);
rank_Ob == size(A,1)

%Decomposizione osservabile / non osservabile
[Ao, Bo, Co, To] = obsvf(A, B, C);

%Trasformazione del sistema nella nuova base
Abar = To \ A * To;
Bbar = To \ B;
Cbar = C * To;

%Dimensione del sottospazio non osservabile
r = size(A,1) - rank(obsv(A,C));

%Estrazione del sottosistema NON osservabile
Ano = Abar(end-r+1:end, end-r+1:end);
Bno = Bbar(end-r+1:end, :);