clc; clear all; close all;
%% Do math
syms w1 w2 w3 dt theta position
r = 0.25;
l = 0.3;
x_g = sym(zeros(3,3));
x_g(1,:) = [0, 1, l];
x_g(2,:) = [-sqrt(sym(3))/2, (-1)/2, l];
x_g(3,:) = [sqrt(sym(3))/2, (-1)/2, l];
x_r = inv(x_g);
% v_eq = v1+v2+v3;
% w_eq = cross(l1,v1)/norm(l1) + cross(l2,v2)/norm(l2) +cross(l3,v3)/norm(l3);
u = sym(zeros(3,1));
u(1) = w1;
u(2) = w2;
u(3) = w3;
v = r * x_r * u;
theta_n = theta + v(3)*dt;
rotation_m = [cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,(1)];
position_n = position + (rotation_m*v) * dt;

linearizedMat = sym(zeros(3,3));
% 
% X-axis
linearizedMat(1,1) = diff(position_n(1), position);
linearizedMat(1,2) = 0;
linearizedMat(1,3) = diff(position_n(1), theta);
% 
% Y-axis
linearizedMat(2,1) = 0;
linearizedMat(2,2) = diff(position_n(2), position);
linearizedMat(2,3) = diff(position_n(2), theta);
% 
% Rotation.
linearizedMat(3,1) = 0;
linearizedMat(3,2) = 0;
linearizedMat(3,3) = diff(theta_n, theta);
% 
% Meas model. 
linearizedMeas = sym(eye(3,3));










