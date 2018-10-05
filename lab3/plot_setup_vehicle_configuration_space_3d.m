%% Visualize configuration space in 3D
function plot_setup_vehicle_configuration_space_3d(sp, state_occupancy)
    hold all

    xs = sp.dim_centers{1};
    ys = sp.dim_centers{2};
    zs = sp.dim_centers{3};
    minmax_x = minmax(xs) + [-1 1]*sp.dim_width(1)/2;
    minmax_y = minmax(ys) + [-1 1]*sp.dim_width(2)/2;

    volume = state_occupancy(:,:,:,1);
    volume = permute(volume, [2 1 3]);
    data = smooth3(volume,'box');
    patch(isocaps(xs, ys, zs, data,.5),...
       'FaceColor','interp','EdgeColor','none');
    p1 = patch(isosurface(xs, ys, zs, data,.5),...
       'FaceColor','blue','EdgeColor','none');
    isonormals(xs, ys, zs, data,p1)
    view(3) 
    grid on
    xlabel('x (meter)')
    ylabel('y (meter)')
    zlabel('orientation \theta (radians)')
    axis equal
    axis vis3d
    xlim(minmax_x)
    ylim(minmax_y);

    camlight headlight; 
    colormap jet
    lighting gouraud
end