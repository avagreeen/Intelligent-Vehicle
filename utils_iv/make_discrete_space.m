function sp = make_discrete_space(dim_min, dim_max, dim_size, circular_dims)
    % ensure column vectors
    dim_min = dim_min(:);
    dim_max = dim_max(:);
    dim_size = dim_size(:);
    
    D = numel(dim_size);
    assert(D == numel(dim_min));
    assert(D == numel(dim_max));

    % determine which dimensions are bounded, and which are circular
    mask_dim_modulo = false(D,1);
    if exist('circular_dims', 'var')
        mask_dim_modulo(circular_dims) = true;
    end
    mask_dim_clamped = ~mask_dim_modulo;

    % create cell indices
    dim_width = (dim_max - dim_min) ./ (dim_size);

    % create cell-array storing per dimension the cell centers
    dim_centers = arrayfun( ...
        @(d) linspace(dim_min(d) + dim_width(d)/2, dim_max(d) - dim_width(d)/2, dim_size(d)), ...
        1:D, ...
        'UniformOutput', false);

    % precompute stepsize for N-d to 1-d conversion
    dim_steps = cumprod([1; dim_size(1:end-1)]);
    num_cells = prod(dim_size);
    
    cell_centers = cell(1, D);
    [cell_centers{:}] = ind2sub(dim_size, 1:num_cells);
    cell_centers = arrayfun(@(d) dim_centers{d}(cell_centers{d}), 1:D, 'UniformOutput', false);
    cell_centers = cat(1,cell_centers{:});    
    
    %% create result struct
    sp = struct;
    sp.dim_size = dim_size;
    sp.dim_width = dim_width;
    sp.dim_centers = dim_centers;
    sp.cell_centers = cell_centers;
    sp.num_cells = num_cells;
    sp.num_dims = D;
    sp.input_to_1d = @input_to_1d;
    sp.input_to_1d_NaN = @input_to_1d_NaN;
    sp.input_to_nd = @input_to_nd;
    sp.input_to_nd_NaN = @input_to_nd_NaN;
    sp.map_nd_to_1d = @map_nd_to_1d;
    sp.map_1d_to_nd = @map_1d_to_nd;
    sp.adjacent_1d = @adjacent_1d;
    sp.move_1d = @move_1d;
    
    %% -- helper functions ---
    function idx1 = input_to_1d(inp)
        idxn = input_to_nd(inp);
        idx1 = map_nd_to_1d(idxn);
    end
    
    function idx1 = input_to_1d_NaN(inp)
        idxn = input_to_nd_NaN(inp);
        idx1 = map_nd_to_1d(idxn);
    end

    function data_idx = input_to_nd(data_in)
        % first, convert all dimensions to [0, 1] range
        data_idx = data_in;
        data_idx = bsxfun(@minus, data_idx, dim_min); % subtract minimum value 
        data_idx = bsxfun(@rdivide, data_idx, dim_max-dim_min); % divide by minmax range
        data_idx = min(max(0, data_idx), 1); % clamp in case min/max bounds are within data range

        % now change [0, 1] range to [1, ... dim_size] range
        data_idx = bsxfun(@times, data_idx, dim_size);
        data_idx = ceil(data_idx);
        data_idx(data_idx == 0) = 1; % index was 0 possible if data_in <= table_min
    end

    function data_idx = input_to_nd_NaN(data_in)
        % first, convert all dimensions to [0, 1] range
        data_idx = data_in;
        data_idx = bsxfun(@minus, data_idx, dim_min); % subtract minimum value 
        data_idx = bsxfun(@rdivide, data_idx, dim_max-dim_min); % divide by minmax range
        data_idx(data_idx < 0) = NaN;
        data_idx(data_idx > 1) = NaN;

        % now change [0, 1] range to [1, ... dim_size] range
        data_idx = bsxfun(@times, data_idx, dim_size);
        data_idx = ceil(data_idx);
        data_idx(data_idx == 0) = 1; % index was 0 possible if data_in <= table_min
    end

    function idxn = fix_nd_bounds(idxn)
        % CLAMP the dimensions in mask_dim_clamped
        idxn_clamp = min(max(idxn, 1), dim_size);
        idxn(mask_dim_clamped, :) = idxn_clamp(mask_dim_clamped, :);

        % circular dimensions
        idxn_mod = mod(idxn - 1, dim_size) + 1;  
        idxn(mask_dim_modulo, :) = idxn_mod(mask_dim_modulo, :);
    end

    function idxn = valid_nd_bounds(idxn)
        % DISCARD the dimensions in mask_dim_clamped
        out_of_bounds = idxn < 1 | idxn > repmat(dim_size,1,size(idxn,2));
        out_of_bounds(mask_dim_modulo, :) = false; % ignore circular domains
        idxn(:,any(out_of_bounds,1)) = [];
        
        % circular dimensions
        idxn_mod = mod(idxn - 1, repmat(dim_size,1,size(idxn,2))) + 1;  
        idxn(mask_dim_modulo, :) = idxn_mod(mask_dim_modulo, :);
    end

    % N-d to 1-d index conversion
    function idx1 = map_nd_to_1d(idxn)
        idx1 = sum(bsxfun(@times, dim_steps, idxn-1), 1)+1;
    end

    function idxn = map_1d_to_nd(idx1, dims)
        idxc = cell(1,D);
        [idxc{:}] = ind2sub(dim_size, idx1);
        idxn = [idxc{:}]';
        if nargin > 1
            % only return request dimensions
            idxn = idxn(dims,:);
        end
    end

    function nidxs = adjacent_1d(idx1, dims)
        if nargin < 2; dims = 1:D; end
        
        idxn = map_1d_to_nd(idx1);

        nidxs = [];
        for dim = dims
            a_idxn = [idxn, idxn]; % D x 2
            a_idxn(dim,:) = a_idxn(dim,:) + [-1, 1];
            a_idxn = valid_nd_bounds(a_idxn);
            nidxs = [nidxs map_nd_to_1d(a_idxn)];
        end
    end

    function next_idx1 = move_1d(idx1, dim_deltas)
        idxn = map_1d_to_nd(idx1);
        idxn = bsxfun(@plus, idxn, dim_deltas);
        next_idxn = valid_nd_bounds(idxn);
        next_idx1 = map_nd_to_1d(next_idxn);
    end
end