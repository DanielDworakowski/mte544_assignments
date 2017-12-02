function [ v, delta, done ] = carrot_controller(start, goal, pose)
r = 1.9;
k = 1.0;
tol = 0.9;
done = 0;
delta_max = deg2rad(30);

p0 = start;
p1 = goal;
p = [pose(1), pose(2)];
theta = pose(3);

e = (p1-p0);

t0 = dot(e, p - p0) / dot(e, e);

px = p0 + t0*e;

rv = px + e;

s = rv - p;

delta = max(-delta_max,min(delta_max, angleWrap(k*(atan2(s(2),s(1))) - (theta))));
v=3;
% dif = p1-p0;
% if delta > deg2rad(30)
%   delta = deg2rad(30);
% %    v=2.2;
% end
% if delta < -deg2rad(30)
%   delta = -deg2rad(30);
% %    v=2.2;
% end
if(norm(p-p1) < tol)
  done = 1;
end
% disp(["velocity: ", num2str(v), "   delta: ", num2str(delta), "    done: ", num2str(done)]);
% pose
% sz = 20;
% figure
% hold
% axis([-100, 100, -100, 100]);
% scatter(p0(1),p0(2), sz,1,'filled');
% text(p0(1)-0.1,p0(2)-0.1, 'p0');
% 
% scatter(e(1),e(2), sz,8,'filled');
% text(e(1)-0.1,e(2)-0.1, 'e');
% 
% scatter(pc(1),pc(2), sz,7,'filled');
% text(pc(1)-0.1,pc(2)-0.1, 'pc');
% 
% scatter(p1(1),p1(2), sz,2,'filled');
% text(p1(1)-0.1,p1(2)-0.1, 'p1');
% 
% scatter(p(1),p(2), sz,3,'filled');
% text(p(1)-0.1,p(2)-0.1, 'p');
% 
% scatter(rv(1),rv(2), sz,8,'filled');
% text(rv(1)-0.1,rv(2)-0.1, 'rv');
% 
% scatter(p(1)+0.1*cos(theta),p(2)+0.1*sin(theta), sz,3,'filled');
% text(p(1)+0.1*cos(theta)-0.1,p(2)+0.1*sin(theta)-0.1, 'w');
% 
% scatter(s(1),s(2), sz,4,'filled');
% text(s(1)-0.1,s(2)-0.1, 's');
% 
% scatter(0,0, sz,4,'filled');
% text(-0.1,-0.1, 'O');

end

