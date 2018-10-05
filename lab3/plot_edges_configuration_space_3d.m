function plot_edges_configuration_space_3d(cspace, from_idxs, to_idxs, varargin)
    assert(all(size(from_idxs) == size(to_idxs)));

    from_xyz = cspace.cell_centers(1:3,from_idxs(:))';
    to_xyz = cspace.cell_centers(1:3,to_idxs(:))';

    [N, D] = size(from_xyz);
    
    pos = cat(3, from_xyz, to_xyz, NaN(size(to_xyz)));
    pos = reshape(permute(pos, [3 1 2]), [N*3, D]);

    plot3(pos(:,1), pos(:,2), pos(:,3), varargin{:});
end