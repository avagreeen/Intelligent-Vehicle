function kfs = run_multiple_kfs(kfs, sensor, measurements)

    J = numel(kfs); % number of Kalman Filters
    T = numel(measurements); % number of timesteps
    
    % iterate over all T time steps
    for t = 1:T
        % get measured distances at the current time step
        dists = measurements(t).dists; 
        
        % convert the measured distances of the sensor to x,y coordinates
        meas_pos = sensor.dist_to_pos(dists);

        % measurments the equal maximum distance of the sensor
        %   or probably not actual object detections.
        % Only keep the measurements that are below this range
        mask = (dists < sensor.max_range);
        meas_pos = meas_pos(:,mask);

        %% Predict step
        for j = 1:J
            % predict step for the j-th Kalman Filter
            kfs(j) = kf_predict_step(kfs(j));
        end
        
        %% Update step

        % In this function, we will assume that all given Kalman Filters
        % are 'active'. Later, when we start adding and terminating tracks,
        % some KFs might not be active anymore (i.e. terminated).
        % The associate_measurments_to_tracks function uses this information
        % to know which KFs it can use.
        % For now, we will say that all J Kalman Filters can be used.
        is_active = true(1,J);
        
        % determine for all R measurements to which Kalman Filter they
        % belong (if any). Gating is also considerd in the assignment.
        assignment = associate_measurments_to_tracks(meas_pos, kfs, is_active);

        for j = 1:J
            %  update the j-th KF with each of its assigned measurements
            for r = find(assignment == j)
                meas_r = meas_pos(:,r);
                
                kfs(j) = kf_update_step(kfs(j), meas_r);
            end
        end
            
    end

end