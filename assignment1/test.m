clear all;
clc;
close all;
%% Setup the env.
T = 15;
dt = 0.1;
%% Question 1
% a) Going Straight
w1 = -1.5 * ones(T / dt, 1);
w2 = 2 * ones(T / dt, 1);
w3 = 1 * ones(T / dt, 1);
multiRateKalmanFilter([w1], [w2], [w3], T);