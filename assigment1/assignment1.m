clear all;
clc;
close all;
%% Setup the env.
T = 15;
dt = 0.1;
%% Question 1
% a) Going Straight
w1 = -1 * ones(T / dt, 1);
w2 = 1 * ones(T / dt, 1);
w3 = 0 * ones(T / dt, 1);
[pose, theta] = simulateRobotVec([w1], [w2], [w3], dt, true);

figure('Name','Question 1 a)');
subplot(2,1,1)       % add first plot in 2 x 1 grid
plot(pose(:,1),pose(:,2));
title('Question 1 a) Position State')
subplot(2,1,2)       % add second plot in 2 x 1 grid
plot(theta);
title('Question 1 a) Theta State')

% b) Turning in Place
w1 = 0.1 * ones(T / dt, 1);
w2 = 0.1 * ones(T / dt, 1);
w3 = 0.1 * ones(T / dt, 1);
[pose, theta] = simulateRobotVec([w1], [w2], [w3], dt, true);

figure('Name','Question 1 b)');
subplot(2,1,1)       % add first plot in 2 x 1 grid
plot(pose(:,1),pose(:,2));
title('Question 1 b) Position State')
subplot(2,1,2)       % add second plot in 2 x 1 grid
plot(theta);
title('Question 1 b) Theta State')
%% Question 2
% a) w = -1.5 , 2.0 , 1.0
w1 = -1.5 * ones(T / dt, 1);
w2 = 2.0 * ones(T / dt, 1);
w3 = 1.0 * ones(T / dt, 1);
[pose, theta] = simulateRobotVec([w1], [w2], [w3], dt, true);

figure('Name','Question 2 a)');
subplot(2,1,1)       % add first plot in 2 x 1 grid
plot(pose(:,1),pose(:,2));
title('Question 2 a) Position State')
subplot(2,1,2)       % add second plot in 2 x 1 grid
plot(theta);
title('Question 2 a) Theta State')

% b) Going Straight
w1 = -1.0 * ones(T / dt, 1);
w2 = 1.0 * ones(T / dt, 1);
w3 = 0.0 * ones(T / dt, 1);
[pose, theta] = simulateRobotVec([w1], [w2], [w3], dt, true);

figure('Name','Question 2 b)');
subplot(2,1,1)       % add first plot in 2 x 1 grid
plot(pose(:,1),pose(:,2));
title('Question 2 b) Position State')
subplot(2,1,2)       % add second plot in 2 x 1 grid
plot(theta);
title('Question 2 b) Theta State')

% c) 2m Diameter Circle
w1 = 1 * ones(T / dt, 1);
w2 = 0.9478 / 0.25 * ones(T / dt, 1);
w3 = -0.1978 / 0.25 * ones(T / dt, 1);
[pose, theta] = simulateRobotVec([w1], [w2], [w3], dt, true);

figure('Name','Question 2 c)');
subplot(2,1,1)       % add first plot in 2 x 1 grid
plot(pose(:,1),pose(:,2));
title('Question 2 c) Position State')
subplot(2,1,2)       % add second plot in 2 x 1 grid
plot(theta);
title('Question 2 c) Theta State')
% d) Sprial
w1 = 1 * ones(T / dt, 1);
t = linspace(0,T / 6, T / dt);
w2 = (t./2 + (-(15625*(4.*t + 5).*(4.*t - 11))/749956).^(1/2)/2 - 1/8) ./ 0.25;
w3 = (t./2 - (-(15625*(4.*t + 5).*(4.*t - 11))/749956).^(1/2)/2 - 1/8) ./ 0.25;
[pose, theta] = simulateRobotVec([w1], [w2], [w3], dt, true);

figure('Name','Question 2 d)');
subplot(2,1,1)       % add first plot in 2 x 1 grid
plot(pose(:,1),pose(:,2));
title('Question 2 d) Position State')
subplot(2,1,2)       % add second plot in 2 x 1 grid
plot(theta);
title('Question 2 d) Theta State')
 
%% Question 3
% a) Going Straight GPS Magnetometer
w1 = -1.0 * ones(T / dt, 1);
w2 = 1.0 * ones(T / dt, 1);
w3 = 0.0 * ones(T / dt, 1);
[positions_state, thetas_state, pos_gps, thetas_mag] = simulateRobotVec([w1], [w2], [w3], dt, true);

figure('Name','Question 3 a)');
subplot(2,1,1)       % add first plot in 2 x 1 grid
plot(pos_gps(:,1),pos_gps(:,2));
title('Question 3 a) Position GPS')
subplot(2,1,2)       % add second plot in 2 x 1 grid
plot(thetas_mag);
title('Question 3 a) Theta Magnetometer')
%% Question 6
% a) Multirate Kalman Filter
T = 15;
dt = 0.1;
w1 = -1.5 * ones(T / dt, 1);
w2 = 2 * ones(T / dt, 1);
w3 = 1 * ones(T / dt, 1);
multiRateKalmanFilter([w1], [w2], [w3], T);