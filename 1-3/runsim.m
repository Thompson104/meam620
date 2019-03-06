close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
if false  
    map = load_map('map1.txt', 0.1, 1.0, 0.25);
    start = {[0.0, -4.9, 0.2]};
    stop = {[8.0, 18.0, 3.0]};
elseif false
    map = load_map('map2.txt', 0.2, 0.5, 0.25);
    start = {[0.2, 10, 2.0]};
    stop = {[5.0, 9.0, 3.0]};
elseif false
    map = load_map('map3.txt', 0.25, 0.5, 0.25);
    start = {[2.0, 2, 1.0]};
    stop = {[17.0, 0.0, 3.0]};
else 
    map = load_map('mymap.txt', 0.25, 0.25, 0.4);
    start = {[0.0, 0.0, 0.0]};
    stop = {[25.5, 5.0, 5.0]};
end

nquad = length(start);
for qn = 1:nquad
    path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
end
if nquad == 1
    %plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end

%% Generate trajectory
disp('Generating Trajectory ...');
trajectory_generator([], [], map, path{1});


%% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
