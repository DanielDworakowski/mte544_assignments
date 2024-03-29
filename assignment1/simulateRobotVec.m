function [ positions_state, thetas_state, pos_gps, thetas_mag ] = simulateRobotVec(w1, w2, w3, dT, noise)

r = 0.25;
l = 0.3;

x_g = zeros(3,3);
x_g(1,:) = [0, 1, l];
x_g(2,:) = [-sqrt(3)/2, (-1)/2, l];
x_g(3,:) = [sqrt((3))/2, (-1)/2, l];
x_r = inv(x_g);

position = [0;0;0];
theta = 0;

pos_meas = [0;0;0;];
theta_meas = 0;

positions_state = [];
thetas_state = [];

pos_gps = [];
thetas_mag = [];

for index = 1:numel(w1)
  w = r * [w1(index); w2(index); w3(index)];
  %
  % Current speeds.
  v = x_r * w;
  %
  % Simulate the motion.
  rotation_m = [cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,(1)];
  position = position + (rotation_m*v) * dT;
  %
  % Calculate the rotation. 
  theta = theta + v(3) * dT;
  if noise
    theta = normrnd(theta,(0.1*pi/180));
  end
  %   
  % Add noise to the motion.
  if noise
    position(1) = normrnd(position(1),0.01);
    position(2) = normrnd(position(2),0.01);
  end
  %
  % Generate the measurements. 
  if noise
    pos_meas(1) = normrnd(position(1),0.5);
    pos_meas(2) = normrnd(position(2),0.5);
    thetas_meas = normrnd(theta + 9.7 * pi / 180.0,10 * pi / 180);
  else
    pos_meas(1) = normrnd(position(1),0.0);
    pos_meas(2) = normrnd(position(2),0.0);
    thetas_meas = normrnd(theta + 9.7 * pi / 180.0, 0.0);
  end
  
  positions_state = [positions_state; position'];
  thetas_state = [thetas_state; theta];
  pos_gps = [pos_gps; pos_meas'];
  thetas_mag = [thetas_mag; thetas_meas];
  
end

end

