% Euler integration of motion model
function states = euler_integrate_motion(init_state, dt, path_length, motion_model, steering_profile)

    % Assuming a constant velocity profile:
    % - estimate the duration to travel the given path length
    % - estimate traveled distance at each step
    velocity = init_state.v; 
    T = (path_length / velocity);
    nsteps = ceil(T / dt);
    ts = (1:nsteps)*dt;
    traveled_dists = velocity * ts;

    t = 0; % t is time in seconds since start
    
    % set initial state
    state = init_state;

    % start simulation
    states = [];
    for step = 1:nsteps
        t = t + dt;
        dt = min(dt, T-t); % last time step might use smaller dt

        % assuming fixed velocity profile, so 0 acceleration
        u_acc = 0;

        % Here we 
        %   - compute the steering control u_steer, and then
        %   - simulate the next state using the motion model
        
        % Compute the steering control u_steer
        %  by calling the steering profile using the fraction
        %  of path traveled s / s_f.
        %  Note that the current traveled distance s is given
        %  by traveled_dists(step), and the path_length give the
        %  total length s_f.
        % ----------------------
        %  YOUR CODE GOES HERE! 
        % ----------------------


        % Now we simply update the vehicle state using the given
        %   motion_model, the control inputs u_acc and u_steer,
        %   and the time difference dt.
        % ----------------------
        %  YOUR CODE GOES HERE! 
        % ----------------------

        
        % append result
        states = [states, state];
    end

end