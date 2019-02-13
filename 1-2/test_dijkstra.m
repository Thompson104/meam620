close all;
map = load_map('sample_maps/map1.txt', 0.3, 0.3, 0.2);
[path, num_expanded] = dijkstra(map, [5,6,6], [6,0,1], 0);
plot_path(map, path);
