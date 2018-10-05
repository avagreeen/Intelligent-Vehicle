% --------------------------------------------------------
% Intelligent Vehicles Lab Assignment
% --------------------------------------------------------
% Julian Kooij, Delft University of Technology
% Based on [Ferguson et. al. 2008] "Motion Planning in Urban Environments"

% clear the workspace
clear all;
close all;
clc;

% setup paths
startup_iv

%% -- define road segment --

% define the road curvature radius (meter)
rradius = 60; % as rradius goes to inf, the road becomes more straight
road = setup_trajectory_scenario(rradius);

fprintf(' -- road segment properties --\n');
fprintf('  segment length: %d meter\n', road.rlength);
fprintf('curvature radius: %d meter\n', road.rradius);
fprintf('      lane width: %d meter\n', road.rwidth);

% plot the road
figure(1);
clf
plot_setup_trajectory_planning(road)

% initial state
init_state = struct( ...
    'x', 0, 'y', 0, ...
    'theta', 0e-1*pi, 'kappa', 1/rradius, ...
    'v', 5, 'a', 0);

% plot initial state
sfigure(1);
delete(findobj('Tag', 'vehicle'))
plot_vehicle_state(init_state);
drawnow

%% Exercise 2.1: Define a spline steering profile
%
% You will need to complete the code in
%     make_steering_profile

k0 = -1; k1 = .2; k2 = -.3; % some values for clear illustration

steering_profile = make_steering_profile(k0, k1, k2);

% plot the profile, and animate the effect on the wheels
figure(2);
clf;
animate_steering_profile_plot(steering_profile, k0, k1, k2);


%% Define goal position and angle along the road
% The goal vehicle state is defined with respect the road curve.

lat_offset = 0; % lateral offset (in meters) how many meters from centerline ?
long_frac = 1; % longitudinal offset (fraction between 0 and 1)
goal_state = define_goal_state(road, long_frac, lat_offset, init_state);

% visualize problem
sfigure(1);
delete(findobj('Tag', 'vehicle'))
delete(findobj('Tag', 'track'))
delete(findobj('Tag', 'candidate'))
% plot initial state
plot_vehicle_state(init_state, 'Color', 'b', 'DisplayName', 'initial state');
% plot goal state
plot_vehicle_state(goal_state, 'Color', 'g', 'DisplayName', 'goal state');
legend_by_displayname('Location', 'SouthEast');
drawnow

%% Exercise 2.2 & 2.3: Euler integration of the motion model
% Let's take a look at how the steering control parameters affect the path.
% 
% You will need to complete the code in
%     euler_integrate_motion (Exercise 2.2)
%     curv_param_state_predict (Exercise 2.3)

% set initial parameter estimates (will later be redefined through optimization)
k0 = init_state.kappa;
k1 = 1/2; % <-- change this
k2 = -1/4; % <-- change this
path_length = 20; % <-- change this

% simultation time steps in seconds, i.e. 10 Hz
dt = 1e-1;

% -- define the vehicle dynamics --
% See Algorithm 3 [Ferguson, 2008]
% state: [x, y, theta, kappa, v, a]
%   x     : vehicle positional x
%   y     : vehicle positional y
%   theta : vehicle orientation 
%   kappa : curvature
%   v     : velocity
%   a     : acceleration
% control:
%   u_acc   : acceleration command
%   u_steer : steering angle command

motion_model = @(state, u_acc, u_steer, dt) struct( ...
    'x',     state.x + state.v * sin(state.theta) * dt, ...
    'y',     state.y + state.v * cos(state.theta) * dt, ... 
    'theta', state.theta + state.v * state.kappa * dt, ...
    'kappa', u_steer, ...
    'v',     state.v + state.a * dt, ...
    'a',     u_acc ...
);

% Change this once you understand how the k parameters affect the vehicle
% path.
USE_CURV_PARAM_STATE_PREDICT = false;

if ~USE_CURV_PARAM_STATE_PREDICT
    % Perform Euler integration

    % define steering profile
    steering_profile = make_steering_profile(k0, k1, k2);

    % simulate dynamics: reutrn the vehicle state at each step
    states = euler_integrate_motion( ...
        init_state, ...
        dt, ...
        path_length, ...
        motion_model, ...
        steering_profile ...
    );

else
    %% Do the same as the previous code block, but now using a single
    %   function.

    % put almost all information into a single struct,
    % this makes calling curv_param_state_predict easier
    config = struct;
    config.dt = dt;
    config.motion_model = motion_model;
    config.init_state = init_state;

    % put the control parameters in a single vector
    params = [k1, k2, path_length];

    states = curv_param_state_predict(config, params);
end

% animate resulting vehicle states
figure(1);
clf
plot_setup_trajectory_planning(road)
plot([states.x], [states.y], 'b-', 'Tag', 'track');
for j = 1:numel(states)
    delete(findobj('Tag', 'vehicle'))
    plot_vehicle_state(states(j));
    pause(.01)
end

%% Exercise 2.4: Optimize control parameters
% Now the goal is to find those control parameters that make the vehicle
% reach the goal position and orientation.
%
% You will need to complete the code in
%     optimize_control_parameters

% initial parameters
k1 = init_state.kappa;
k2 = init_state.kappa;
path_length = road.rlength;
init_params = [k1, k2, path_length];

% run non-linear optimization algorithm
params = optimize_control_parameters(config, goal_state, init_params);
show params

% recompute optimal state sequence for optimized control parameters
states = curv_param_state_predict(config, params);

% plot and animate vehicle states
figure(1)
delete(findobj('Tag', 'track'))
delete(findobj('Tag', 'vehicle'))
plot([states.x], [states.y], 'b-', 'Tag', 'track', 'DisplayName', 'obtained path');
legend_by_displayname('Location', 'SouthEast');
drawnow

for j = 1:numel(states)
    delete(findobj('Tag', 'vehicle'))
    plot_vehicle_state(states(j));
    pause(.01)
end

params_center_line = params;

%% Create candidate trajectories with different lateral offsets
% Using the trajectory optimization, we can here create multiple
% candidate trajectories with different lateral offsets in a simple
% for-loop.

num_candidates = 7; % number of candidate trajectories, each with different lateral offset

lat_offsets = linspace(-5, 5, num_candidates); % how many meters from centerline ?
init_params = params_center_line; % initialize from parameters found in first optimization round

candidates = [];
for lat_offset = lat_offsets(:)'
    
    % define new goal for given lateral offset
    goal_state_lat = define_goal_state(road, 1, lat_offset, init_state);

    % run optimization
    params = optimize_control_parameters(config, goal_state_lat, init_params);

    % recompute optimal state sequence
    states = curv_param_state_predict(config, params);

    % put some information of the candidate together in a struct
    candidate = struct;
    candidate.lat_offset = lat_offset;
    candidate.params = params;
    candidate.states = states;
    
    % append the results to the list of candidates
    candidates = [candidates, candidate];
end

% plot candidate tracks
figure(1);
clf
plot_setup_trajectory_planning(road)
plot_vehicle_state(init_state)
for candidate = candidates
    states = candidate.states;
    plot([states.x], [states.y], 'Color', [1 1 1]*.6, 'Tag', 'candidate');
end
legend off

%% Exercise 2.5: Compute trajectory cost with respect to other obstacles
% We now consider various scenarios with another moving vehicle.
% Each scenario defines a variation with a different vehicle moving
%  on (or off!) the road, potentially crossing the path of our own
%  vehicle.
%
% You will need to complete the code in
%     compute_trajectory_cost

% select scenario 1 to 6
scenario_idx = 1; % <-- ** change this **

obstacle_states = trajplanning_obstacle_scenario(road, scenario_idx);

% evaluate candidate tracks
for cidx = 1:numel(candidates)
    candidate = candidates(cidx);
    
    % compute the cost of the candidate by comparing the proposed
    % trajectory to the predicted path of the obstacle,
    % and by considering the lateral offset of the path
    [total_cost, cost_per_timestep] = compute_trajectory_cost( ...
        obstacle_states, ...
        candidate.states, ...
        candidate.lat_offset);

    show cidx total_cost
  
    % add the computed cost to the stored candidate information
    candidates(cidx).cost_per_timestep = cost_per_timestep;
    candidates(cidx).total_cost = total_cost;
end

% plot candidate trajectories color by cost
sfigure(1);
delete(findobj('Tag', 'costs'))
delete(findobj('Tag', 'candidate'))
cost_range = minmax([candidates.total_cost]);
for candidate = candidates
    total_cost = candidate.total_cost;
    states = candidate.states;
    
    % use the to visualize good (green) and bad (red) trajectories
    alpha = (total_cost - cost_range(1)) ./ diff(cost_range);
    plot([states.x], [states.y], 'Color', [alpha 1-alpha 0], 'Tag', 'candidate');
end

% -- inspect a single candidate --
% determine optimal candidate
[best_cost, best_cidx] = min([candidates.total_cost]);
cidx = best_cidx;
%cidx = 1; % DEBUG: you could manually select any of the trajectories here

candidate = candidates(cidx);
states = candidate.states;
fprintf('-- selected candidate %d : total_cost = %.3f --\n', cidx, candidate.total_cost);

% -- animate --
sfigure(1);
delete(findobj('Tag', 'costs'))
%scatter([states.x], [states.y], 15, -log(candidate.cost_per_timestep), 'filled', 'Tag', 'costs'); % show costs

T = min(numel(states), numel(obstacle_states));
for t = 1:T;
    obstacle = obstacle_states(t);
    state = states(t);

    % update ego-vehicle and obstacle
    delete(findobj('Tag', 'vehicle'))
    delete(findobj('Tag', 'obstacle'))
    plot_vehicle_state(state);
    plot_vehicle_state(obstacle, 'Color', 'b', 'Tag', 'obstacle');
    pause(.01)
end
