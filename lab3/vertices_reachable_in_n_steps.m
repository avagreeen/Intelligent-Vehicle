function [to_idxs, from_idxs] = vertices_reachable_in_n_steps(start_idx, reachable, nsteps)
to_idxs = []; % list of 'reachable' nodes within n steps
from_idxs = []; % for each node in to_idxs, a node to get to it

% the list of nodes to explore, initially only the given start node
expand_idxs = [start_idx];

% iterate over the given number of (time) steps
for step = 1:nsteps
    
    new_idxs = [];
    for from_idx = expand_idxs
        
        % reachable nodes in graph from current node 'from_idx'
        nidxs = reachable{from_idx};
        
        % repeat 'from_idx' for each reachable nodes
        from_idx = repmat(from_idx, 1, numel(nidxs));
        
        % ok, store the (from, to) pairs
        from_idxs = [from_idxs from_idx];
        to_idxs = [to_idxs nidxs];
        
        % keep track of newly reached nodes
        new_idxs = [new_idxs nidxs];
    end
    
    % remove any previously reached nodes form the new nodes,
    %  and use these to expand further in the new time step
    expand_idxs = setdiff(new_idxs, from_idxs);
end

end