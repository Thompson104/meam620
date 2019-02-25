close all;
s = 0.25;
%map = load_map('sample_maps/map0.txt', 0.25, 0.25, 0.5);
%[path, num_expanded] = dijkstra(map, [2,2,2], [14,4,2], 0);
%map = load_map('sample_maps/map1.txt', 0.25, 0.25, 0.5);
%[path, num_expanded] = dijkstra(map, [1,-3,1], [1,17,2], 0);
map = load_map('sample_maps/empty.txt', 1, 2, 0);
[path, num_expanded] = dijkstra(map, [1,1,0], [4,4,0], 0);

d = diff(path);
all(vecnorm(d')==s)
plot_path(map, path);
