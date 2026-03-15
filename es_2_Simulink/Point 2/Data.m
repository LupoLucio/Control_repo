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

% Initial conditions
theta0_deg = 60;
theta0dot_deg = 0;

% Radians conversion
theta0 = deg2rad(theta0_deg);
theta0dot = deg2rad(theta0dot_deg);