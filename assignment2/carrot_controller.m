function [ v, delta, done ] = carrot_controller(start, goal, pose)
delta_max = deg2rad(30);
velocity = 3;
k = 4;
tol = 0.35;
done = 0;
position = [pose(1) pose(2)];
end_point = goal;
start_point = start;
traj_angle = atan2(end_point(2) - start_point(2), end_point(1) - start_point(1));

[crosstrack_error, next_point] = distanceToLineSegment(start_point,end_point,position);

% Calculate steering angle
delta = max(-delta_max,min(delta_max, angleWrap(traj_angle - pose(3))+ atan2(-k*crosstrack_error,velocity)));
v = velocity;

if(norm(position-goal) < tol)
  done = 1;
end
end

