function dist = vehicle_to_road_distance(states, rx, ry)
% Compute overall Euclidean distance between road and vehicle track

    if ~exist('dtw')
        warning('No Dynamic Time Warping function available')
        dist = NaN;
        return;
    end

    veh_xy = [[states.x]; [states.y]];
    road_xy = [rx(1,:); ry(1,:)];

    if verLessThan('matlab', 'R2016a')
        % NOTE:
        % Here I'm using the MEX library dynamic_time_warping_v2.1 by Quan Wan.
        % http://nl.mathworks.com/matlabcentral/fileexchange/43156-dynamic-time-warping--dtw-
        dist = dtw(veh_xy', road_xy'); 
    else
        % However, Matlab 2016a has its DTW own implementation, might need
        % adaptation
        dist = dtw(veh_xy, road_xy); 
    end
end