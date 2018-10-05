function plot_setup_mot(sensor)

    % for some reason, AXIS EQUAL gives deprecation warnings :-s
    %   can't do anything about it now, only distracting
    warning off MATLAB:nargchk:deprecated
    
    % setup the figure, draw the vehicle and the sensors
    axis equal
    axis([-15 15 -5 20]);
    hold all;
    grid on
    xlabel('lateral - x (meters)')
    ylabel('longitudinal - y (meters)')

    % plot sensor rays
    delete(findobj('Tag', 'sensor'))
    plot_sensor_rays(sensor);
    
    % define vehicle state, located at origin
    veh = struct( 'x', 0, 'y', 0, 'theta', 0, 'kappa', 0, 'v', 0, 'a', 0);    
    delete(findobj('Tag', 'vehicle'))
    plot_vehicle_state(veh);

end