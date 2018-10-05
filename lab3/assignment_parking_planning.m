% --------------------------------------------------------
% Intelligent Vehicles Lab Assignment
% --------------------------------------------------------
% Julian Kooij, Delft University of Technology

% clear the workspace
clear all;
close all;
clc;

% setup paths
startup_iv

%% define the configuration space, and reachable vertices
[cspace, reachable] = parking_scenario;

V = cspace.num_cells; % total number of cells
D = cspace.num_dims; % number of dimensions (D = 4)
states = cspace.cell_centers; % 4D state at each of the cell centers

% -- define environment --
% for simplicity, we 'draw' the occupied environment in a small grid
occupancy_2d = zeros(32, 32); % 32 x 32 is the same gridsize used in the sp struct
occupancy_2d(20:26, 27:32) = 1;
occupancy_2d(20:26, 1:6) = 1;
occupancy_2d = imresize(occupancy_2d, cspace.dim_size(1:2), 'nearest');

% compute which cells in the discrete space are occupied, given the static
% obstacles in the world
state_occupancy = determine_occupied_cells(cspace, occupancy_2d);

% this little utility function will convert a vertex idx to a struct that
% can be used in plot_vehicle_state
vehstruct_from_idx = @(idx) struct( ...
    'x', states(1,idx), 'y', states(2,idx), 'theta', states(3,idx), ...
    'kappa', states(4,idx), 'v', 0, 'a', 0);

% print some information on the size in the discrete configuration space
fprintf('-- discrete configuration space --\n');
fprintf('dimension 1 (x pos) has %d cells\n', cspace.dim_size(1));
fprintf('dimension 2 (y pos) has %d cells\n', cspace.dim_size(2));
fprintf('dimension 3 (theta) has %d cells\n', cspace.dim_size(3));
fprintf('dimension 4 (omega) has %d cells\n', cspace.dim_size(4));
fprintf('In total, the state space has %d cells\n', V);

%% Exercise 2.1: visualize the vehicle in the discrete configuration space
% We can define the vehicle state by its grid cell in the 4D configuration space,
%   and map it to a unique single vertex idx.

% define a grid cell in the discrete state space
idx_4d = [1 1 1 1]'; % <-- change this (note its a column vector)

% convert given 4D grid indices to single cell id
idx = cspace.map_nd_to_1d(idx_4d);
fprintf('the cell at grid location [%d, %d, %d, %d]\n', idx_4d(1), idx_4d(2), idx_4d(3), idx_4d(4));
fprintf('    has id %d\n', idx);

% Show the top-down map of the environment
sfigure(1);
clf
plot_setup_groundplane_2d(cspace, occupancy_2d);
plot_vehicle_state(vehstruct_from_idx(idx), 'Color', 'b', 'Tag', 'start', 'DisplayName', 'vehicle');
legend_by_displayname

% Show a 3D plot of the path in the (x,y,theta) configuration space
sfigure(2);
clf;
plot_setup_vehicle_configuration_space_3d(cspace, state_occupancy)
plot_vehicle_state_configuration_space_3d(cspace, idx, 'bd', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'DisplayName', 'vehicle')

% determine which vertices are reachable within n steps,
%   and visualize these paths in the 3D configuration space
nsteps = 6;
[to_idxs, from_idxs] = vertices_reachable_in_n_steps(idx, reachable, nsteps);
plot_edges_configuration_space_3d(cspace, from_idxs, to_idxs, ...
    '.-r', 'Tag', 'reachable', ...
    'DisplayName', sprintf('reachble within %d steps', nsteps));

legend_by_displayname('Location', 'NorthWest')

%% Planning problem definition
% Select the planning problem here. All problems have the vehicle start
% at the same origin, but define different goal positions in the
% environment.

planning_problem_idx = 1; % <-- change this

% define the vehicle start position in the created discrete space
start = cspace.map_nd_to_1d([7 1 1 1]');
switch planning_problem_idx
    case 1, goal = cspace.input_to_1d([8 4.5 270/180*pi 0]');
    case 2, goal = cspace.input_to_1d([10 1 180/180*pi 0]');
    case 3, goal = cspace.input_to_1d([10 1 0 0]');
    case 4, goal = cspace.input_to_1d([10 9 180/180*pi 0]');
    case 5, goal = cspace.input_to_1d([7 5 180/180*pi 0]');
end

% plot initial state
sfigure(1)
delete(findobj('Tag', 'vehicle'))
delete(findobj('Tag', 'start'))
delete(findobj('Tag', 'goal'))
plot_vehicle_state(vehstruct_from_idx(start), 'Color', 'b', 'Tag', 'start', 'DisplayName', 'start');
plot_vehicle_state(vehstruct_from_idx(goal), 'Color', 'g', 'Tag', 'goal', 'DisplayName', 'goal');
drawnow
legend_by_displayname

% check if everything is correct
if state_occupancy(start); error('illegal start state'); end
if state_occupancy(goal); error('illegal goal state'); end

%% Exercise 2.2: plan the path using A-star in the configuration space
fprintf('planning the path ...\n');

% Euclidean distance function between vertices
vertex_dist = @(idx1, idx2) ...
    sqrt( sum(bsxfun(@minus, states(:,idx1), states(:,idx2)).^2, 1) )';

% perform graph search for shortest path
[path, info] = search_shortest_path(V, start, goal, reachable, vertex_dist, vertex_dist);

% done, print some info
fprintf('duration     : %.2f seconds\n', info.duration);
fprintf('iterations   : %d steps\n', info.iterations);
fprintf('distance     : %.3f km\n', info.path_length/1000.);
fprintf('vertex count : %d vertices\n', numel(path));

% -- animate the resulting path --
figure(1);
delete(findobj('Tag', 'path'))
plot(states(1,path), states(2,path), '.-', 'Color', [1 1 1]*.7, 'Tag', 'path')

for t = 1:numel(path)
    idx = path(t);
    
    % plot initial state
    sfigure(1);
    delete(findobj('Tag', 'vehicle'))
    plot_vehicle_state(vehstruct_from_idx(idx));
    pause(.1)
end

%% -- Configuration space 3D plot --
% Show a 3D plot of the path in the (x,y,theta) configuration space
figure(2);
clf;
plot_setup_vehicle_configuration_space_3d(cspace, state_occupancy)
plot_vehicle_state_configuration_space_3d(cspace, start, 'bd', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'DisplayName', 'start')
plot_vehicle_state_configuration_space_3d(cspace, goal, 'gd', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'DisplayName', 'goal')
plot_vehicle_state_configuration_space_3d(cspace, path, 'r.-', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'path');
legend_by_displayname
