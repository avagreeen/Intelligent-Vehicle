function plot_setup_trajectory_planning(road)

rx = road.rx;
ry = road.ry;

% plot the road segments
plot(rx(1,:), ry(1,:), 'k--', 'LineWidth', 2, 'DisplayName', 'lane center'); % center
hold all
plot(rx(2,:), ry(2,:), 'k-'); % left side of our lane (e.g. lane seperation)
plot(rx(3,:), ry(3,:), 'k-'); % right side of our lane
plot(rx(4,:), ry(4,:), 'k-'); % side of other lane (even more left)
xlabel('world x (meter)')
ylabel('world y (meter)')

axis([-10 10 0 40])
warning off MATLAB:nargchk:deprecated % axis equal seems to throw warnings in certain matlab versions ...
axis equal;
grid on
