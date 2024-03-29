% Extended Kalman filter example
clear;clc; close all;

%% Create AVI object
makemovie = 0;
if(makemovie)
    vidObj = VideoWriter('ekf.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

% Discrete time step
dt = 0.1;

% Initial State
x0 = [0 0 0]';

% Prior
mu = [0 0 0]'; % mean (mu)
S = 0.01^2*eye(3);% covariance (Sigma)
S(3) = (0.1*pi/180.0)^2;

% Discrete motion model
% Ad = [ 1 dt 0 ; 0 1 0; 0 0 1];
Ad = eye(3,3);

R = [.0001 0 0; 0 .0001 0 ; 0 0 .0001];
[RE, Re] = eig (R);

% Measurement model defined below
Q = eye(3,3) * 0.5^2;
Q(3,3) = (10 * pi /180.)^2;

% Simulation Initializations
Tf = 15;
T = 0:dt:Tf;
n = 3;
x = zeros(n,length(T));
x(:,1) = x0;
m = length(Q(:,1));
y = zeros(m,length(T));
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));


r = 0.25;
l = 0.3;
x_g = zeros(3,3);
x_g(1,:) = [0, 1, l];
x_g(2,:) = [-sqrt(3)/2, (-1)/2, l];
x_g(3,:) = [sqrt((3))/2, (-1)/2, l];
x_r = inv(x_g);

w1 = -1.5;
w2 = 1;
w3 = 2;
%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    %
    % Current speeds.
    w = r * [w1; w2; w3];
    v = x_r * w;
    %
    % Calculate the rotation.
    dtheta =  v(3) * dt;
    %
    % Simulate the motion.
    rotation_m = [cos(x(3,t-1)),-sin(x(3,t-1)),0;sin(x(3,t-1)),cos(x(3,t-1)),0;0,0,(1)];
    dposition = (rotation_m*v) * dt;
    %     
    % Next state.
    x(:,t) = [x(1,t-1)+dposition(1);
              x(2,t-1)+dposition(2);
              x(3,t-1)+dtheta] + e;
    %
    % Take measurement
    % Select a motion disturbance
    d = sqrt(Q)*randn(m,1);
    % Determine measurement
%     y(:,t) = sqrt(x(1,t)^2 + x(3,t)^2) + d;
    y(:,t) = [x(1,t), x(2,t), x(3,t)].' + d;

    %% Extended Kalman Filter Estimation
    %
    % Linearization.
    [Ad, Ht] = getStateEqsMat(w1, w2, w3, mu(3), dt);
    %
    % Prediction update
    rotation_m = [cos(mu(3)),-sin(mu(3)),0;sin(mu(3)),cos(mu(3)),0;0,0,(1)];
    dposition = (rotation_m * v) * dt;
    dtheta =  w(3) * dt;
    mup =    [mu(1)+dposition(1);
              mu(2)+dposition(2);
              mu(3)+dtheta];
    
    Sp = Ad*S*Ad' + R;

    % Measurement update
    K = Sp*Ht'*inv(Ht*Sp*Ht'+Q);
    mu = mup + K*(y(:,t)-mup);
    S = (eye(n)-K*Ht)*Sp;

    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
%     K_S(:,t) = K;


    %% Plot results
    figure(1); hold on;
    plot(0,0,'bx', 'MarkerSize', 6, 'LineWidth', 2)
    plot([20 -1],[0 0],'b--')
    plot(x(1,2:t),x(2,2:t), 'ro--')
    plot(mu_S(1,2:t),mu_S(2,2:t), 'bx--')
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    title('True state and belief')
    axis equal
    axis([-0.25 3 -1 2])
    if (makemovie) writeVideo(vidObj, getframe(gca)); end
    figure(2); clf; hold on;
    plot(0,0,'bx', 'MarkerSize', 6, 'LineWidth', 2)
    plot([20 -1],[0 0],'b--')
    plot(x(1,2:t),x(2,2:t), 'ro--')
    plot(mu_S(1,2:t),mu_S(2,2:t), 'bx--')
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    title('True state and belief')
    axis equal
    axis([-0.25 3 -1 2])
end
figure(1); saveas(gcf, './ekfAllElipse.png');
figure(2); saveas(gcf, './ekfLastElipse.png');
if (makemovie) close(vidObj); end

