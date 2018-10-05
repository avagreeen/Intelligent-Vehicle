function target_state = define_goal_state(road, long_frac, lat_offset, init_state)

[target_x, target_y, target_theta] = make_road_xy( ...
    road.rradius, ...
    road.rlength*long_frac, ...
    lat_offset);

target_state = init_state;
target_state.x = target_x;
target_state.y = target_y;
target_state.theta = target_theta;

end