function [ positions_state, thetas_state ] = simulateRobot(w1, w2, w3)

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


time = 15;
period = 1/10;
position = [0;0;0];
theta = 0;
positions_state = [];
thetas_state = [];
for t = 1:period:time
%% 
%   sub_vals = [1,1,1,(1)/4,(3)/10];
  v = v1+v2+v3;
  w = cross(l1,v1)/norm(l1) + cross(l2,v2)/norm(l2) +cross(l3,v3)/norm(l3);

  rotation_m = [cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,(1)];
  position = position + (rotation_m*v') * period;
  theta = theta + w(3)*period;
  theta = normrnd(theta,(0.1*pi/180));
  position(1) = normrnd(position(1),0.01);
  position(2) = normrnd(position(2),0.01);
  
  positions_state = [positions_state; position'];
  thetas_state = [thetas_state; theta];
end


end

