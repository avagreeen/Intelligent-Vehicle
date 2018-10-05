function [objects, T] = mot_scenario(scenario_id)
    if nargin < 1; scenario_id = 1; end

    % number of timesteps
    T = 50;

    % trajectory (x1, y1) of object 1 
    o1_pos = [
        linspace(-13, 9, T); % x1
        linspace(10, 5, T)  % y1
    ];

    % trajectory (x2, y2) if object 2
    o2_pos = [
        linspace(5, -9, T); % x2
        linspace(7, 5, T)  % y2
    ];

    switch scenario_id
        case 1,
            desc = '2 persons, crossing';
            % nothing to change to trajectory

        case 2,
            desc = '2 persons crossing, one stopping';
            t_stop = 30;
            o1_pos(1,t_stop:end) = o1_pos(1,t_stop);
            o1_pos(2,t_stop:end) = o1_pos(2,t_stop);
           
        otherwise,
            error('incorrect scenario id "%d" given', scenario_id)
    end
    
    objects = struct;
    objects(1).pos = o1_pos;
    objects(1).plot_color = 'b';
    objects(2).pos = o2_pos;
    objects(2).plot_color = 'r';

    fprintf('** MULTI-target scenario %d: %s\n', scenario_id, desc);
end
