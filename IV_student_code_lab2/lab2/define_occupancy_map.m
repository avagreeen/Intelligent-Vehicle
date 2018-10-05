function map = define_occupancy_map(frac_to_remove)
if nargin < 1; frac_to_remove = 0.7; end % default fraction to remove

% define grid dimensions
xs = linspace(-25, 25, 32);
ys = linspace(-5, 45, 32);
[X, Y] = meshgrid(xs, ys);

% 'draw' in the grid where walls/roads are located
grid = ones(numel(ys), numel(xs));
grid(X >= -1 & X < 2) = 0;
grid(Y >= 20 & Y <= 25) = 0;
grid(X >= -15 & X <= -10 & Y < 20) = 0;
grid(X <= -15 & Y >= 7 & Y <= 12) = 0;

% freespace is where ther are no roads
freespace = [X(grid == 0), Y(grid == 0)]';

% add some more structure to the map (don't count this as freespace)
%  by removing random parts
grid = grid .* (rand(size(grid)) > frac_to_remove);

obstacles = [X(grid > 0), Y(grid > 0)]';

map = struct;
map.xs = xs;
map.ys = ys;
map.grid = grid;
map.freespace = freespace;
map.obstacles = obstacles;

end