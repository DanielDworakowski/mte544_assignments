function [] = multiRateKalmanFilter(w1, w2, w3, Tf)
%% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('multiratekf.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

% Discrete time step
dt = 0.1;

% Prior
mu = zeros(3,1); % mean (mu)
S = 0.01*eye(3);% covariance (Sigma)

% Continuous motion model (2-D)
r = 0.25;
l = 0.3;

v1_hat = [0,1,0];
v2_hat = [-sqrt(3)/2,(-1)/2,0];
v3_hat = [sqrt((3))/2,(-1)/2,0];

l1 = [1,0,0]*l;
l2 = [(-1)/2,sqrt((3))/2,0]*l;
l3 = [(-1)/2,-sqrt((3))/2,0]*l;


% Simulation Initializations
T = 0:dt:Tf;
n = 3;
x = zeros(n,length(T));
x(:,1) = zeros(n,1);
mp = 2;
mv = 3;
y = zeros(mv,length(T));
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));
position = [0;0;0];
theta = 0;
i=1;
%% Main loop
for t=2:length(T)
    v1 = v1_hat*w1(t-1)*r;
    v2 = v2_hat*w2(t-1)*r;
    v3 = v3_hat*w3(t-1)*r;
    i = i+1;
    %
    % Current speeds.
    v = v1+v2+v3;
    w = cross(l1,v1)/norm(l1) + cross(l2,v2)/norm(l2) +cross(l3,v3)/norm(l3);

    %theta = theta + w(3) * dt;
    theta = x(3,t-1);
    Ad = [1 0 0;
          0 1 0;
          0 0 1];
    
    Bd = [cos(theta)*dt -sin(theta)*dt 0;
          sin(theta)*dt  cos(theta)*dt 0;
                   0                 0 dt];
    
    R = [.0001 0 0; 0 .0001 0 ; 0 0 (0.1*pi/180)^2];
    [RE, Re] = eig (R);

    % Measurement model
    Cv = eye(3);
    Cp = zeros(2,3);
    Cp(1,1) = 1.0;
    Cp(2,2) = 1.0;
    D = zeros(2,2);
    Qv = eye(3,3) * 0.5^2;
    Qv(3,3) = (10 * pi /180.)^2;
    Qp = eye(2,2) * 0.01^2;
    [QpE, Qpe] = eig (Qp);
    [QvE, Qve] = eig (Qv);
    u = [v(1); v(2); w(3)];
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update state
    x(:,t) = Ad*x(:,t-1)+ Bd*u + e;

    % Take measurement
    % Select a measurement disturbance and determine measurement
    if (mod(t,10)==0)
        d = QpE*sqrt(Qpe)*randn(mp,1);
        y([1 2],t) = Cp*x(:,t) + d;
    else
        d = QvE*sqrt(Qve)*randn(mv,1);
        y(:,t) = Cv*x(:,t) + d;
    end
    
    %% Kalman Filter Estimation
    % Prediction update
    Bd = [cos(mu(3))*dt -sin(mu(3))*dt 0;
          sin(mu(3))*dt  cos(mu(3))*dt 0;
                   0                 0 dt];
    mup = Ad*mu + Bd*u;
    Sp = Ad*S*Ad' + R;

    % Measurement update
    if (mod(t,10) == 0)
        K = Sp*Cp'*inv(Cp*Sp*Cp'+Qp);
        mu = mup + K*(y([1 2],t)-Cp*mup);
        S = (eye(n)-K*Cp)*Sp;
    else
        K = Sp*Cv'*inv(Cv*Sp*Cv'+Qv);
        mu = mup + K*(y(:,t)-Cv*mup);
        S = (eye(n)-K*Cv)*Sp;
    end
    
    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    K_S(:,t) = [K(:,1); K(:,2)];
    
    theta = x(3,t);
  
    %% Plot results
    figure(1);clf; hold on;
    plot(x(1,2:t),x(2,2:t), 'ro--')
    if (mod(t,10)==0) plot(y(1,t),y(2,t), 'gx'); end
    %plot(mup_S(3,1:t),mup_S(1,1:t), 'mx--')
    plot(mu_S(1,2:t),mu_S(2,2:t), 'bx--')
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.999999);
    error_ellipse(S_pos,mu_pos,0.9999999);
    title('True state and beliefs')
    legend('State', 'Measurement','Estimate')
    axis([-0.5 4.5 -4 2])
    if (makemovie) writeVideo(vidObj, getframe(gca)); end
    
end
if (makemovie) close(vidObj); end

end
