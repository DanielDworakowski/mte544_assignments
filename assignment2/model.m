clc; clear; close all;
%%Question 1.
T = 20;
dt = 0.1;
ts = 0:dt:T;
init = zeros(3,1);
v = ones(1, T/dt + 1) * 3;
delta = (10 - ts) .* pi / 180.;
X = motionModel(v, delta, nan);
plotState(X, 'q1MotModel');

function [] = plotState(X, name)
  figure;
  plot(X(:,1), X(:,2));
  title('State x vs y position');
  ylabel('y (m)');
  xlabel('x (m)');
  saveas(gcf, strcat(name,'.png'));
end

function [X] = motionModel(vel, delta, init)
  wb = 0.3;
  dt = 0.1;
  sigma = zeros(3,3);
  sigma(1,1) = 0.02;
  sigma(2,2) = 0.02;
  sigma(3,3) = 1 * pi / 180.0;
  d_limit = 30 * pi / 180.0;
  %   
  %   Motion modeling.
  X = zeros(size(vel,2),3);
  vel_local = zeros(3,1);
  vel_global = zeros(3,1);
  if isnan(init)
    init = zeros(1,3);
  end
  %   
  % Enforce limits.
  delta(delta > d_limit) = d_limit;
  delta(delta < -d_limit) = d_limit;
  %   
  % Previous state.
  prevState = init;
  %   
  % Iterate through all of the indicies.
  for index = 1:size(vel, 2)
    %     
    % Robot motion dt
    v_cmd = vel(index);
    d_cmd = delta(index);
    c = tan(d_cmd)/ wb;
    omega = v_cmd * c;
    vel_local(1) = v_cmd;
    vel_local(3) = omega;
    % 
    % Global motion.
    theta = prevState(3); 
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    vel_global(1:2) = R * vel_local(1:2);
    vel_global(3) = omega;
    dX = vel_global .* dt;
    X(index, :) = prevState + dX.';
    prevState = X(index, :);
  end

end
