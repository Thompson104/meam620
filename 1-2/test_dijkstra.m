close all;
map = load_map('sample_maps/map0.txt', 0.25, 0.25, 0.5);
[path, num_expanded] = dijkstra(map, [2,2,2], [14,4,2], 0);
plot_path(map, path);
