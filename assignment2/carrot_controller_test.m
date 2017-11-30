clear all;
clc;
close all;

start = [-5,2];
goal = [1,3];
pose = [1,4,0.2];
[v, delta] = carrot_controller(start, goal, pose);