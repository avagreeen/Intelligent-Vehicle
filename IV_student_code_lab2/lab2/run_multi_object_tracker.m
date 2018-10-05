% Tracking multiple objects, creating and terminating tracks if needed.
%
% The Multi-Object Tracker performs as follows:
% Initially, there are no tracks.
% Then each time step, the tracker
%   1. GET MEASUREMENTS: retrieve the measurements of the new time step
%   2. PREDICT TRACKS: perform the KF predict step for all tracks
%   3. DATA ASSOCIATION: assign measurements to tracks (also do gating)
%   4. UPDATE TRACKS: use the assigned measurement for the KF update step
%   5. CREATE TRACKS: init new track by placing KF on unassigned measurements.
%        5 1/2. if needed, perform new DATA ASSOCIATION with added track
%   6. TERMINATE TRACKS: if a track has no measurement associated for
%         several time step, terminate it
%
function kfs = run_multi_object_tracker(sensor, measurements, track_termination_threshold)

    T = numel(measurements); % number of timesteps

    %% setup datastructures
    kfs = []; % will contain list of Kalman Filters for all tracked objects
    is_active = false(1,0); % indicates which KFs are active (terminated tracks are 'inactive')
    dt_without_update = zeros(1,0); % how many time steps without any associated measurement? Used to terminate tracks.

    % loop over all time instances
    for t = 1:T
        %% -- GET MEASUREMENTS --
        % get measures for this time step, convert the valid ones to (x,y) coordinates
        dists = measurements(t).dists;    
        dists(dists >= sensor.max_range) = NaN; % invalidate max-range measurements
        meas_pos = sensor.dist_to_pos(dists); % get x,y coordinates of measurements

        %% -- PREDICT TRACKS --
        % predict all active trackers
        for j = find(is_active);
            kfs(j) = kf_predict_step(kfs(j));
            dt_without_update(j) = dt_without_update(j) + 1;
        end

        %% -- DATA ASSOCIATION --    
        % determine which measurement belongs to which track
        assignment = associate_measurments_to_tracks(meas_pos, kfs, is_active);

        %% -- UPDATE TRACKS --
        % update tracks with assigned observations
        for j = find(is_active); % loop over active tracks
            for r = find(assignment == j) % loop over all assigned measurements

                % Kalman Filter UPDATE step
                kfs(j) = kf_update_step(kfs(j), meas_pos(:,r));

                % reset its 'timesteps without measurements' counter
                dt_without_update(j) = 0;
            end
        end

        %% -- CREATE TRACKS --
        % initialize new tracks for unassigned observations

        % determine indices of unassigned but valid measurements
        unassigned_rs = find(isnan(assignment) & ~isnan(dists));
        while ~isempty(unassigned_rs)

            % get one of the unassigned measurements
            r = unassigned_rs(1);
            meas_r = meas_pos(:,r);

            % intialize a tracker on top of this measurement
            m_init = zeros(4,1);
            m_init(1:2) = meas_r; %initialize x y position
            S_init = diag([1e0, 1e0, 1e-1 1e-1]);

            new_kf = kf_init(m_init, S_init);
            new_kf.ts(end) = t;

            % add the Kalman Filter to the list of trackers
            kfs = [kfs, new_kf];
            is_active = [is_active, true];
            dt_without_update = [dt_without_update, 0];

            % re-check assignment of all unassigned measurements
            assignment = associate_measurments_to_tracks(meas_pos, kfs, is_active);
            unassigned_rs = find(isnan(assignment) & ~isnan(dists));
            if ismember(r, unassigned_rs);
                % hmmm, the r-th measurement is still unassigned,
                % which is not a good sign. Maybe the gating threshold
                % is too strict?
                break;
            end

        end

        %% -- TERMINATE TRACKS --
        % invalidate tracks that have not been updated for too long
        is_active(dt_without_update >= track_termination_threshold) = false;

    end % go to next time step
end
