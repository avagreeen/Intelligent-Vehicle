function plot_sensor_rays(sensor, varargin)

    % (x,y) coordinates of the sensor
    center_pos = sensor.ray_center;
    
    % (x,y) coordinates of end positions of all M rays
    end_pos = sensor.dist_to_pos(sensor.max_range);
    
    % number of rays
    M = size(end_pos, 2);
    for m = 1:M
        plot([center_pos(1) end_pos(1,m)], [center_pos(2) end_pos(2,m)], ...
            'Color', [1 1 1]*.8, ...
            'Tag', 'sensor', varargin{:});
    end
end