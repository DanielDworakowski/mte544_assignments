clear all;
clc;
close all;

start = [76.6000   65.9000];
goal = [14.8000   38.3000];
pose = [78.4103   65.5023    0.5562];
[v, delta] = carrot_controller(start, goal, pose);