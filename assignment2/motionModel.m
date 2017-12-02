function [X] = motionModel(vel, delta, init, noise)
  wb = 0.3;
  dt = 0.1;

  d_limit = 30 * pi / 180.0;
  %   
  %   Motion modeling.
  X = zeros(size(vel,2),3);
  vel_local = zeros(3,1);
  vel_global = zeros(3,1);
  dX = zeros(3,1);
  
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
    c = tan(d_cmd) / wb;
    omega = v_cmd * c;

    % 
    % Global motion.
    theta = prevState(3); 

    %

    if (abs(c) < 0.001)
%      disp('straight')
      R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
      vel_local(1) = v_cmd;
      vel_global(1:2) = R * vel_local(1:2);
      vel_global(3) = omega;
      dX = vel_global .* dt;
      dX = prevState + dX.';
      dX(3) = mod(dX(3) + pi,2*pi)-pi;
    else
      dTheta = c * dt * v_cmd;
      r = 1 / c;
      x_rot = prevState(1) - r * sin(theta);
      y_rot = prevState(2) + r * cos(theta);
      dX(1) = x_rot + r * sin(dTheta + theta);
      dX(2) = y_rot - r * cos(dTheta + theta);
      dX(3) = mod(theta + dTheta + pi,2*pi)-pi;
    end
    if noise
      dX(1) = normrnd(dX(1),0.01^2);
      dX(2) = normrnd(dX(2),0.01^2);
      dX(3) = normrnd(dX(3), (pi / 180)^2);
    end
    X(index, :) = dX;
    prevState = X(index, :);
    
  end

end