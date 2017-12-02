clear all;
clc;
close all;
%% Question 2
clear all;
clc;
close all;
pose = [0 0 0];
start_m = [0 0];
goal_m = [20 0];
done = 0;
poses = [];
while ~done
[ v, delta, done ] = carrot_controller(start_m, goal_m, pose);
pose = motionModel(v, delta, pose, false)
poses = [poses; pose];
end

start_m = [pose(1) pose(2)];
goal_m = [20 5];
done = 0;
while ~done
[ v, delta, done ] = carrot_controller(start_m, goal_m, pose);
pose = motionModel(v, delta, pose, false)
poses = [poses; pose];
end

start_m = [pose(1) pose(2)];
goal_m = [0 5];
done = 0;
while ~done
[ v, delta, done ] = carrot_controller(start_m, goal_m, pose);
pose = motionModel(v, delta, pose, false)
poses = [poses; pose];
end

start_m = [pose(1) pose(2)];
goal_m = [0 0];
done = 0;
while ~done
[ v, delta, done ] = carrot_controller(start_m, goal_m, pose)
pose = motionModel(v, delta, pose, false)
poses = [poses; pose];
end
plot(poses(:,1), poses(:,2));
%% Question 3.
clear all;
clc;
close all;
[map, start, goal, res, xMax, yMax] = getMap(false);

tic;

% Get milestones
nS = 2200;
% 
% Max sample locations
sampleMaxX = xMax - 1;
sampleMaxY = yMax - 1;
start(1:2) = start(1:2) / res;
goal(1:2) = goal(1:2) / res;
samples = round([sampleMaxX*rand(nS,1) + 1, sampleMaxY*rand(nS,1) + 1]);
idx = sub2ind(size(map),samples(:,1), samples(:,2));
keep = ~map(idx);
milestones = [start(1:2); goal; samples(find(keep==1),:)];
% 
% Plot samples.
figure(1); clf; hold on;
colormap('gray');
imagesc(1-map');
plot(start(1), start(2), 'ro', 'MarkerSize',10, 'LineWidth', 3);
plot(goal(1), goal(2), 'gx', 'MarkerSize',10, 'LineWidth', 3 );
axis equal
plot(samples(:,1),samples(:,2),'k.');
plot(milestones(:,1),milestones(:,2),'m.');
nM = length(milestones(:,1));
disp('Time to generate milestones');
toc;

% Attempt to add closest p edges
tic;
p = 18;
e = zeros(nM,nM);
D = zeros*ones(nM,nM);
for i = 1:nM
  % Find closest neighbours
  for j = 1:nM
      d(j) = norm(milestones(i,:)-milestones(j,:));
  end
  [d2,ind] = sort(d);
  % Check for edge collisions (no need to check if entire edge is
  % contained in obstacles as both endpoints are in free space)
  for j=1:p
    cur = ind(j);
    if (i<cur)
       x1 = milestones(i,1);
       y1 = milestones(i,2);
       x2 = milestones(cur,1);
       y2 = milestones(cur,2);
       [x, y] = bresenham(x1, y1, x2, y2);
       idx = sub2ind(size(map), x, y);
       isCollided = any(map(idx));
       if (~isCollided)
         e(i, cur) = 1;
         e(cur, i) = 1;
         plot([milestones(i,1) milestones(cur,1)],[milestones(i,2) milestones(cur,2)],'m');

       end
       
%       if (~CheckCollision(milestones(i,:),milestones(cur,:), obsEdges))
%           e(i,cur) = 1;
%           e(cur,i) = 1;
%           plot([milestones(i,1) milestones(cur,1)],[milestones(i,2) milestones(cur,2)],'m');
%       end
    end
  end
end

disp('Time to connect roadmap');
toc;

% Find shortest path
tic;
[sp, sd] = shortestpath(milestones, e, 1, 2);
for i=1:length(sp)-1
    plot(milestones(sp(i:i+1),1),milestones(sp(i:i+1),2), 'go-', 'LineWidth',3);
end
disp('Time to find shortest path');
toc;
%
pose = start.*res;
poses = [];

length(sp)
for i=1:length(sp)-1
  x = milestones(sp(i:i+1),1);
  y = milestones(sp(i:i+1),2);
  start_m = [x(1) y(1)].* res;
  goal_m = [x(2) y(2)].* res;
  done = 0;
  while ~done
    [ v, delta, done ] = carrot_controller(start_m, goal_m, pose);
    pose = motionModel(v, delta, pose, false)
    poses = [poses; pose];
  end
end
disp('DONE');
plot(poses(:,1) ./ res, poses(:,2) ./ res);
hold off;

%% Question 1.
T = 20;
dt = 0.1;
ts = 0:dt:T;
init = zeros(3,1);
v = ones(1, T/dt + 1) * 3;
delta = (10 - ts) .* pi / 180.;
X = motionModel(v, delta, nan, true);
plotState(X, true, 'q1MotModel');
