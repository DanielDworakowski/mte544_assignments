clc; clear all; close all;
%% Do math
syms w1 w2 w3 dt theta position
r = 0.25;
l = 0.3;

v1_hat = [0,1,0];
v2_hat = [-sqrt(3)/2,(-1)/2,0];
v3_hat = [sqrt((3))/2,(-1)/2,0];

v1 = v1_hat*w1*r;
v2 = v2_hat*w2*r;
v3 = v3_hat*w3*r;

l1 = [1,0,0]*l;
l2 = [(-1)/2,sqrt((3))/2,0]*l;
l3 = [(-1)/2,-sqrt((3))/2,0]*l;

v_eq = v1+v2+v3;
w_eq = cross(l1,v1)/norm(l1) + cross(l2,v2)/norm(l2) +cross(l3,v3)/norm(l3);
theta_n = theta + w_eq(3)*dt;
rotation_m = [cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,(1)];
position_n = position + (rotation_m*v_eq') * dt;

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










