% Data association: assign measurments to tracks
function assignment = associate_measurments_to_tracks(meas_pos, kfs, is_active)

    J = numel(kfs); % number of filters
    R = size(meas_pos, 2); % number of measurement rays
    
    % assignment(r) = j means that measurement r is assigned to KF j
    assignment = NaN(1, R);

    % this array can be used to keep track of the best gating score
    %   per measurement r, found over all active Kalman Filters.
    best_scores = inf(1, R);
    
    if J == 0
        % no filters present
        return
    end
    
    % some (many) measurements may have been rejected (e.g. because they
    %  are equal to the maximum sensor range) and have been replaced by
    %  NaN values. Here we get a list of indices of the measurements
    %  that have not been rejected.
    valid_rs = find(~any(isnan(meas_pos),1));
    
    % indices of active KFs
    active_js = find(is_active);
    
    % evaluate log likelihood of all observations per tracker
    for r = valid_rs % iterate over valid measurements ...
        for j = active_js; % iterate over active KFs ...
            % get 2D location of r-th measurement
            meas_r = meas_pos(:,r);
            
            % Here we call test_gating_score to perform the gating test.
            % We also obtain the gating score.
            %
            % If the test succeeds, do the following, in pseudo-code
            %
            %     if score is better than best_score(r):
            %          best_score(r) = score
            %          assignment(r) = j
            
            % ----------------------
            %  YOUR CODE GOES HERE! 
            % ----------------------
            [is_ok, score] = test_gating_score(kfs(j), meas_r);
            if is_ok
                if(score>best_scores(r))
                    best_scores(r)=score;
                    assignmnet(r)=j;
                end
            end
        end
    end
end