% Plot the measurements at time t in the (x,y) plane.
% Usage:
%    plot_current_measurements(sensor, measurements)
%    plot_current_measurements(sensor, measurements, 'Color', 'r')
%
% SEE ALSO plot_measurements
function plot_current_measurements(sensor, measurement, varargin)
    % get the measured distances at time t
    meas_dists = measurement.dists; % the distances at the current time step

    % get x,y coordinates of measurements
    meas_pos = sensor.dist_to_pos(meas_dists);
    
    if 0
        % only show measurements that are NOT the maximum range
        m_mask = (meas_dists < sensor.max_range);
        meas_pos = meas_pos(:,m_mask);
    end
    
    % plot measurements
    plot(meas_pos(1,:), meas_pos(2,:), 'kx', 'Tag', 'meas', varargin{:});
end