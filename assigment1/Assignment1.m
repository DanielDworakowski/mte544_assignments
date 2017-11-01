clear all;
clc;
close all;
%% Setup the env.
T = 15;
dt = 0.1;
%% Simulation 1
w1 = -1.5;
w2 = 2;
w3 = 1;
% 
% Run sim.
[pose, theta] = simulateRobot(w1, w2, w3);
figure;
plot(pose(:,1),pose(:,2));
figure;
plot(theta);
%% Simulation 2 - Straight line
w1 = -1 * ones(T / dt, 1);
w2 = 1 * ones(T / dt, 1);
w3 = 0 * ones(T / dt, 1);
[pose, theta] = simulateRobotVec(w1, w2, w3, dt);
figure; plot(pose(:,1),pose(:,2)); title('Position x,y');
figure; plot(theta); title('Theta(t)');
%% Simulation 3 - Circle 2m diameter
w1 = 1 * ones(T / dt, 1);
w2 = 0.9478 / 0.25 * ones(T / dt, 1);
w3 = -0.1978 / 0.25 * ones(T / dt, 1);
[pose, theta] = simulateRobotVec(w1, w2, w3, dt);
figure; plot(pose(:,1),pose(:,2)); title('Position x,y');
figure; plot(theta); title('Theta(t)');
%% Simulation 4 - Spiral
w1 = 1 * ones(T / dt, 1);

t = linspace(0,T / 6, T / dt);
w2 = (t./2 + (-(15625*(4.*t + 5).*(4.*t - 11))/749956).^(1/2)/2 - 1/8) ./ 0.25;
w3 = (t./2 - (-(15625*(4.*t + 5).*(4.*t - 11))/749956).^(1/2)/2 - 1/8) ./ 0.25;
 
[pose, theta, pose_gps, theta_mag] = simulateRobotVec(w1, w2, w3, dt);
figure; plot(pose_gps(:,1), pose_gps(:,2)); title('Position x,y');
figure; plot(theta); title('Theta(t)');

























