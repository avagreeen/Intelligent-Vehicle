function plot_vehicle_state_configuration_space_3d(sp, idxs, varargin)

% show first position
coords = sp.cell_centers(1:3, idxs);
plot3(coords(1,:), coords(2,:), coords(3,:), varargin{:});

end