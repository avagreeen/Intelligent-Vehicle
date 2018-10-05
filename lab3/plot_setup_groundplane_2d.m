function plot_setup_groundplane_2d(sp, occupancy_2d)

minmax_x = minmax(sp.dim_centers{1}) + [-1 1]*sp.dim_width(1)/2;
minmax_y = minmax(sp.dim_centers{2}) + [-1 1]*sp.dim_width(2)/2;

hold all
axis equal;
axis([minmax_x(1) minmax_x(2) minmax_y(1) minmax_y(2)] + [-1 1 -1 1]*2)
grid on
xlabel('x (meter)')
ylabel('y (meter)')

% plot the outline of the 2D groundplane space (i.e. where car center can
% be located
plot(minmax_x([1 1 2 2 1]), minmax_y([1 2 2 1 1]), 'k--', 'DisplayName', 'spatial bounds');

% plot occupancy
[ox,oy] = ind2sub(size(occupancy_2d), find(occupancy_2d)');
ox = sp.dim_centers{1}(ox); % grid cell to spatial coordinates
oy = sp.dim_centers{2}(oy);
scatter(ox, oy, 50, 'ko', 'DisplayName', 'obstacle');

end