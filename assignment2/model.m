clc; clear; close all;
%%Question 1.
T = 20;
dt = 0.1;
ts = 0:dt:T;
init = zeros(3,1);
v = ones(1, T/dt + 1) * 3;
delta = (10 - ts) .* pi / 180.;
X = motionModel(v, delta, nan);
plotState(X, true, 'q1MotModel');
%% Question 3.
clear
[map, start, goal, res, xMax, yMax] = getMap(false);




tic;

% Get milestones
nS = 1000;
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
p = 20;
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

pose = start.*res;
poses = [];

for i=1:length(milestones)-1
  start_m = [milestones(i,1), milestones(i,2)] .* res;
  goal_m = [milestones(i+1,1), milestones(i+1,2)] .* res;
  done = 0;
  while ~done
    [ v, delta, done ] = carrot_controller(start_m, goal_m, pose);
    pose = motionModel(v, delta, pose);
    poses = [poses pose];
  end
  i
end
plotState(poses,false, "wut");

function [] = plotState(X, newFig, name)
  if (newFig)
    figure;
  end
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
    X(index,3) = mod(X(index,3) + pi,2*pi)-pi;
    prevState = X(index, :);
  end

end

function [map, start, goal, res, xMax, yMax] = getMap(plot)
  I = imread('IGVCmap.jpg');
  map = im2bw(I, 0.7); % Convert to 0-1 image
  map = 1-flipud(map)'; % Convert to 0 free, 1 occupied and flip.
  [xMax,yMax]= size(map); % Map size

  % Robot start position
  res = 0.1;
  start = [40 5 pi];

  % Target location
  goal = [50 10];

  % Plotting
  if plot
    figure(1); clf; hold on;
    colormap('gray');
    imagesc(1-map');
    plot(start(1)/res, start(2)/res, 'ro', 'MarkerSize',10, 'LineWidth', 3);
    plot(goal(1)/res, goal(2)/res, 'gx', 'MarkerSize',10, 'LineWidth', 3 );
    axis equal
  end
end
% 
% From MTE544 git.
function [x y]=bresenham(x1,y1,x2,y2)

%Matlab optmized version of Bresenham line algorithm. No loops.
%Format:
%               [x y]=bham(x1,y1,x2,y2)
%
%Input:
%               (x1,y1): Start position
%               (x2,y2): End position
%
%Output:
%               x y: the line coordinates from (x1,y1) to (x2,y2)
%
%Usage example:
%               [x y]=bham(1,1, 10,-5);
%               plot(x,y,'or');
x1=round(x1); x2=round(x2);
y1=round(y1); y2=round(y2);
dx=abs(x2-x1);
dy=abs(y2-y1);
steep=abs(dy)>abs(dx);
if steep t=dx;dx=dy;dy=t; end

%The main algorithm goes here.
if dy==0 
    q=zeros(dx+1,1);
else
    q=[0;diff(mod([floor(dx/2):-dy:-dy*dx+floor(dx/2)]',dx))>=0];
end

%and ends here.

if steep
    if y1<=y2 y=[y1:y2]'; else y=[y1:-1:y2]'; end
    if x1<=x2 x=x1+cumsum(q);else x=x1-cumsum(q); end
else
    if x1<=x2 x=[x1:x2]'; else x=[x1:-1:x2]'; end
    if y1<=y2 y=y1+cumsum(q);else y=y1-cumsum(q); end
end
end