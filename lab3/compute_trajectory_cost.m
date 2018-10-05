% Evaluate a candidate trajectory by comparing each timestep
function [total_cost, costs_per_timestep] = compute_trajectory_cost(obstacle_states, states, lat_offset)

    % number of timesteps
    nsteps = min(numel(states), numel(obstacle_states));

    costs_per_timestep = NaN(1, nsteps);

    % compute cost for each time step
    for step = 1:nsteps
        % Here we can compute costs_per_timestep(step), e.g. based on the
        % distance between our vehicle state, states(step),
        % and the obstacle state, obstacle_states(step).
        % What do you think would be a good cost function?
        
        % ----------------------
        %  YOUR CODE GOES HERE! 
        % ----------------------

    end

    % Finally, we compute a single cost from the cost per timestep,
    %  and also a cost for the lateral offset.
    % Note that you might need to do some weighting of the different
    % cost terms to get the desired results, depending on how you define
    % the terms.
    
    % ----------------------
    %  YOUR CODE GOES HERE! 
    % ----------------------

    
    assert(numel(costs_per_timestep) == nsteps)
    assert(numel(total_cost) == 1)
end



