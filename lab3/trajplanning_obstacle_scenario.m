function obstacle_states = trajplanning_obstacle_scenario(road, scenario_idx)

rradius = road.rradius;
rlong = road.rlong;

switch scenario_idx
    case 1,
        name = 'normal vehicle on other lane';
        [ox, oy, otheta] = make_road_xy(rradius, rlong(end:-1:1), -4);

    case 2,
        name = 'normal vehicle moving in front';
        [ox, oy, otheta] = make_road_xy(rradius, rlong(1:end)+17, 0);
        otheta = pi+otheta;
        
    case 3,
        name = 'moving in front (too slow)';
        [ox, oy, otheta] = make_road_xy(rradius, rlong(1:end)/2+17, 0);
        otheta = pi+otheta;
        
    case 4,
        name = 'approaching too close to center';
        [ox, oy, otheta] = make_road_xy(rradius, rlong(end:-1:1), -2);
        
    case 5,
        name = 'madman 1 crossing';
        [ox, oy, otheta] = make_road_xy(-rradius, rlong(end:-1:1), -4);
        ox = ox + 9 * sign(rradius);

    case 6,
        name = 'madman 2 crossing';
        [ox, oy, otheta] = make_road_xy(-rradius, rlong(end:-1:1), -4);
        ox = ox + 14 * sign(rradius);

    otherwise,
        error('incorrect obstacle option');
end

fprintf('[scenario %d / 6] %s\n', scenario_idx, name);

% create a struct
obstacle_state_fun = @(t) struct( ...
    'x', ox(t), 'y', oy(t), ...
    'theta', otheta(t)+pi, 'kappa', 0, ...
    'v', 5, 'a', 0);
obstacle_states = arrayfun(obstacle_state_fun, 1:numel(ox));
