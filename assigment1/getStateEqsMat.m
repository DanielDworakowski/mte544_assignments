function [ a_mat meas_mat ] = getStateEqsMat(w1, w2, w3, theta, dt )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

a_mat = eye(3,3);
a_mat(1,3) = dt*(sin(theta)*((3^(1/2)*w2)/12 - (3^(1/2)*w3)/12) + cos(theta)*(w2/12 - w1/6 + w3/12));
a_mat(2,3) = -dt*(cos(theta)*((3^(1/2)*w2)/12 - (3^(1/2)*w3)/12) - sin(theta)*(w2/12 - w1/6 + w3/12));
meas_mat = eye(3,3);

end

