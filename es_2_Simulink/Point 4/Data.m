clear all;
close all;
clc;

% Pendulum parameters
m = 1;      % mass
l = 0.25;      % length
b = 0.05;    % viscous friction
g = 9.81;   % gravity

A = [0 1;
     -g/l -b/(m*(l^2))];

B = [0; 1];

p = [ 1i -1i;
     -1+1i -1-1i;
     -10+1i -10-1i;
     -0.4 -0.5 ];


for i = 1:size(p,1); 
    K{i} = place(A, B, p(i,:));
end

B_new = [0; 0];

C_new = [0 0];

D_new = 0;

for i=1:size(p,1)
    
    sys_1= ss(A-(K{i}')*(B'),B_new,C_new,D_new);
    x0 = [2; 0]';
    t = 0:0.01:10;
    [y1,t1,x1] = initial(sys_1,x0,t);
    
    figure; 
    subplot(2,1,1);
    plot(t1,x1); 
    axis([0 10 -2 2]);
    title('x(t) with friction  initial conditions θ(0) = 2 rad, θdot(0) = 0 rad/s'); 
    grid on;
    
    x0 = [0.1; 0]';
    t = 0:0.01:10;
    [y1,t1,x1] = initial(sys_1,x0,t);
    
    subplot(2,1,2)
    plot(t1,x1); 
    axis([0 10 -0.1 0.1]);
    title('x(t) with friction  initial conditions θ(0) = 0.1 rad, θdot(0) = 0 rad/s'); 
    grid on;

end

