% Create a new sensor, with similar properties as the original sensor,
%   but located at the given position and orientation.
function new_sensor = copy_sensor(sensor, pos_x, pos_y, pos_angle, detect_sigma)
    M = numel(sensor.angles);
    max_range = sensor.max_range;
    detect_prob = sensor.detect_prob;
    
    % minimum vehicle structure
    vehicle = struct( 'x', pos_x, 'y', pos_y, 'theta', pos_angle);

    new_sensor = make_sensor(0, 2*pi, M, max_range, detect_sigma, detect_prob, ...
        vehicle);
end
