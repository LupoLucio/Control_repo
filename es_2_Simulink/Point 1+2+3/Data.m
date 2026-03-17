clear all;
close all;
clc;

% Pendulum parameters
m = 1;      % mass
l = 0.25;      % length
b = 0.05;    % viscous friction
g = 9.81;   % gravity

% Control input
u = 0;   % no control

% pt1: Simulate the system from initial conditions θ(0) = 10 deg, 
% θdot(0) = 0 deg/s. Compare the evolutions of both the linear 
% and non-linear systems. What can you notice?

% Initial conditions
theta0_deg_pt1 = 10;
theta0dot_deg_pt1 = 0;

% Radians conversion
theta0_pt1 = deg2rad(theta0_deg_pt1);
theta0dot_pt1 = deg2rad(theta0dot_deg_pt1);

%Here the approximation is valid since the initial angle is small and 
% the two graphs are identical.


% pt1: Simulate the system from initial conditions θ(0) = 60 deg, 
% θdot(0) = 0 deg/s. Compare the evolutions of both the linear 
% and non-linear systems. What can you notice?

% Initial conditions
theta0_deg_pt2 = 60;
theta0dot_deg_pt2 = 0;

% Radians conversion
theta0_pt2 = deg2rad(theta0_deg_pt2);
theta0dot_pt2 = deg2rad(theta0dot_deg_pt2);

%Here the approximation is no more valid since the initial angle is 
% quite large and the two graphs are a little bit different.