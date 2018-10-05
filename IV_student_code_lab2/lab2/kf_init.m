function kf = kf_init(mu, Sigma)
    % we will use this struct to setup all the 
    % information needed for the Kalman filter, namely:
    %   - its current state (mean and covariance)
    %   - historic state (mean and covariance of previous timesteps)
    %   - the definition of Linear Dynamical System used
    %     later for the predict and update steps.
    
    % check that initial state makes sense
    Dx = numel(mu);
    assert(all(size(mu) == [Dx, 1])); % mu should be column vector
    assert(all(size(Sigma) == [Dx, Dx]));

    kf = struct;
    
    % array which will store the predicted and updated
    %  mean and covariance for each time step, e.g.
    %
    %     mu_preds(:,t) = predicted mu at time t
    %     Sigma_preds(:,:,t) = predicted Sigma at time t    
    %
    % and
    %
    %     mu_upds(:,t) = updated mu at time t
    %     Sigma_upds(:,:,t) = updated Sigma at time t    
    %
    % The matrices are initialized with 0 length,
    % since initially no time has passed, but will be extended
    % in the predict and update steps.
    kf.mu_preds = NaN(Dx, 0);
    kf.Sigma_preds = NaN(Dx, Dx, 0);
    kf.mu_upds = NaN(Dx, 0);
    kf.Sigma_upds = NaN(Dx, Dx, 0);

    % ts (short for timestamps) will store the
    %  time indices corresponding to the predicted and
    %  updated mu, Sigma. Keeping track of the actual
    %  time instance will be useful when we
    %  have multiple Kalman Filters which may have started
    %  at different time instances.
    % initially, zero time steps are recorded
    kf.ts = [];

    % Ok, let's put in the actual 'initial state' for t = 0
    % let's add the initial state to the history
    kf.mu_preds(:,end+1) = mu;
    kf.Sigma_preds(:,:,end+1) = Sigma;
    % we also set the 'updated' values for the new time step
    % equal to the predict ones 
    kf.mu_upds(:,end+1) = mu;
    kf.Sigma_upds(:,:,end+1) = Sigma;
    kf.ts(end+1) = 0; % this is time t = 0
        
    % the dimensionality of the state vector
    kf.Dx = Dx;
    
    % x_t+1 = H x_t + epsilon_t, where noise epsilon_t ~ N(0, Sigma)
    % z_t   = F x_t + eta_t,     where noise eta_t ~ N(0, R)
    noise_var_x_pos = 1e-1; % variance of spatial process noise
    noise_var_x_vel = 1e-2; % variance of velocity process noise
    noise_var_z = 3; % variance of measurement noise for z_x and z_y

    % define the linear dynamics
    %    You should define the F and H matrices
    kf.F = []; % <-- student code should replace this
    kf.H = []; % <-- student code should replace this
    kf.Sigma_x = []; % <-- student code should replace this (hint: see DIAG() function)
    kf.Sigma_z = []; % <-- student code should replace this (hint: see DIAG() function)
    % ----------------------
    %  YOUR CODE GOES HERE! 
    kf.F= [1 0 1 0; 0 1 0 1; 0 0 1 0; 0 0 0 1];
    kf.H= [1 0 0 0; 0 1 0 0];
    kf.Sigma_x = diag([noise_var_x_pos noise_var_x_pos noise_var_x_vel noise_var_x_vel]);
    kf.Sigma_z = diag([noise_var_z noise_var_z]);
    % ----------------------

    % finally, check that everything has correct dimensions
    Dz = size(kf.H, 1); % dimensionality of observations
    assert(all(size(kf.F) == [Dx, Dx]));
    assert(all(size(kf.H) == [Dz, Dx]));
    assert(all(size(kf.Sigma_x) == [Dx, Dx]));
    assert(all(size(kf.Sigma_z) == [Dz, Dz]));
    
end