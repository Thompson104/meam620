map = load_map('sample_maps/no_obstacle_map.txt', 0.5, 0.5, 0.5);
[path, num_expanded] = dijkstra(map, [1,1,1], [0,0,0], 0);
disp('num expanded')
disp(num_expanded)
plot_path(map, path);
