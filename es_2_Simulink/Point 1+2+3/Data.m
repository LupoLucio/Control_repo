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

% Initial conditions pt1
theta0_deg_pt1 = 10;
theta0dot_deg_pt1 = 0;

% Initial conditions pt2
theta0_deg_pt2 = 60;
theta0dot_deg_pt2 = 0;

% Radians conversion pt1
theta0_pt1 = deg2rad(theta0_deg_pt1);
theta0dot_pt1 = deg2rad(theta0dot_deg_pt1);

% Radians conversion pt2
theta0_pt2 = deg2rad(theta0_deg_pt2);
theta0dot_pt2 = deg2rad(theta0dot_deg_pt2);