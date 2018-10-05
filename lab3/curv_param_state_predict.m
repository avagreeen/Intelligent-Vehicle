function states = curv_param_state_predict(config, params)

    init_state = config.init_state;
    motion_model = config.motion_model;
    dt = config.dt;
    
    k0 = init_state.kappa;
    k1 = params(1);
    k2 = params(2);
    path_length = params(3);
    
    % Create the steering profile function here,
    %  and then perform Euler integration.
    %  Use make_steering_profile and euler_integrate_motion.
    % ----------------------
    %  YOUR CODE GOES HERE! 
    % ----------------------

end