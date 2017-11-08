syms w1 w2 w3 t real
% 
% Geometry.
r = 0.25;
l = 0.3;
% 
% Matrix defining the wheel speeds given a rotation and velocity. 
x_g = sym(zeros(3,3));
x_g(1,:) = [0, 1, l];
x_g(2,:) = [-sqrt(sym(3))/2, (-1)/2, l];
x_g(3,:) = [sqrt(sym(3))/2, (-1)/2, l];
x_r = inv(x_g);
% 
% Sub in an assumed wheel velocity.
u = sym(zeros(3,1));
u(1) = w1;
u(2) = w2;
u(3) = w3;
v = r * sym(x_r) * u;
% 
% Assume 1 rotation.
v = subs(v,w3,1);
% 
% Create equations for circle.
f1 = norm(v(1:2)) == 1;
g1 = v(3) == 1;
circ = solve([f1 g1]);
% 
% Equations for a spiral.
f2 = norm(v(1:2)) == t;
g2 = v(3) == 1;
spiral = solve([f2 g2]);