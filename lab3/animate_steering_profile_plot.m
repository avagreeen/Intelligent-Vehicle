% Plot the steering profile, given the three control points, and animate
% the effect on the vehicle.
function animate_steering_profile_plot(steering_profile, k0, k1, k2)

% longitudinal fraction (i.e. how far are we along road)
long_fracs = linspace(0, 1, 100);

% steering angle control inputs
steering_angles = steering_profile(long_fracs);

% utility function to create a vehicle struct given a steering angle
vehicle_state = @(steer_angle) struct( ...
    'x', 0, 'y', 0, ...
    'theta', 0e-1*pi, 'kappa', steer_angle, ...
    'v', 5, 'a', 0);

figure(2);
clf;
subplot(1,2,1)
hax1 = gca;
plot(long_fracs, steering_angles, 'k-', 'DisplayName', 'spline')
hold all
plot( 0, k0, 'r.', 'MarkerSize', 20, 'DisplayName', 'k0')
plot(.5, k1, 'g.', 'MarkerSize', 20, 'DisplayName', 'k1')
plot( 1, k2, 'b.', 'MarkerSize', 20, 'DisplayName', 'k2')
legend_by_displayname
grid on
ylim([-1.1 1.1]*pi)
set(gca, 'YTick', [-1 -.5 0 .5 1]*pi, 'YTickLabel', {'-\pi', '-\pi/2', '0', '\pi/2', '\pi'})
xlabel('fraction of travelled distance')
ylabel('steering angle')

subplot(1,2,2);
hax2 = gca;
hold all
plot_vehicle_state(vehicle_state(steering_angles(1)));
axis([-1 1 -1 1]*2);
axis equal
grid on

for j = 1:numel(long_fracs);
    delete(findobj(hax1, 'Tag', 'steer'))
    delete(findobj(hax2, 'Tag', 'vehicle'))
    
    plot(long_fracs(j), steering_angles(j), '*k', 'Tag', 'steer', 'Parent', hax1);
    state = vehicle_state(steering_angles(j));
    plot_vehicle_state(state, 'Parent', hax2);

    pause(.001)
end

end
