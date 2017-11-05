syms w1 w2 w3 t
% 
% Geometry.
r = 0.25;
l = 0.3;
v1_hat = [0,1,0];
v2_hat = [-sqrt(3)/2,(-1)/2,0];
v3_hat = [sqrt((3))/2,(-1)/2,0];
l1 = [1,0,0]*l;
l2 = [(-1)/2,sqrt((3))/2,0]*l;
l3 = [(-1)/2,-sqrt((3))/2,0]*l;
% 
% Create velocities
v1 = v1_hat*w1*r;
v2 = v2_hat*w2*r;
v3 = v3_hat*w3*r;
%
% Current speeds.
v = v1+v2+v3;
w = cross(l1,v1)/(norm(l1)^2) + cross(l2,v2)/(norm(l2)^2) +cross(l3,v3)/(norm(l3)^2);
% 
% Sub in an assumed wheel velocity.
v_w1 = subs(v, w1, 1);
w_w1 = subs(w, w1, 1);
% 
% Create equations for circle.
f1 = norm(v_w1) == 1;
g1 = w_w1(3) == 1;
circ = solve([f1 g1]);
% 
% Equations for a spiral.
f2 = norm(v_w1) == t;
g2 = g1;
spiral = solve([f2 g2]);