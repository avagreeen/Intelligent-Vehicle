% Draw the occupancy grid, setup axes, etc.
function plot_setup_selfloc(map)
    imagesc(map.xs, map.ys, ~map.grid);
    colormap gray
    axis square

    set(gca, 'YDir', 'normal');
    xlabel('world - x (meters)');
    ylabel('world - y (meters)');
    hold all
end