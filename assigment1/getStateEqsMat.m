function [ a_mat meas_mat ] = getStateEqsMat(w1, w2, w3, theta, dt )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

a_mat = eye(3,3);
a_mat(1,3) = dt*(cos(theta)*(conj(w2)/8 - conj(w1)/4 + conj(w3)/8) + sin(theta)*((3^(1/2)*conj(w2))/8 - (3^(1/2)*conj(w3))/8));
a_mat(2,3) = dt*(sin(theta)*(conj(w2)/8 - conj(w1)/4 + conj(w3)/8) - cos(theta)*((3^(1/2)*conj(w2))/8 - (3^(1/2)*conj(w3))/8));
meas_mat = eye(3,3);

end

