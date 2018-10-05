% Compute measurement log-likelihood for given particle
%
% A particle represent a hypothesis of the vehicle position and
% orientation. Together with the occupancy map, we can use it to determine
% what distances we expect to measure with the sensor,
% and compare it to the actual measurements.
function log_weight = map_measurement_loglik(particle, map, meas, sensor)

veh_x = particle(1);
veh_y = particle(2);
veh_theta = particle(3);

% the actual distances measured (R-dimensional vector z)
z = meas.dists;

% create IDEAL range sensor \hat{z} for the particle 
sensor = copy_sensor(sensor, veh_x, veh_y, veh_theta, 0);
expected_meas = sensor.new_meas();
expected_meas = sensor.observe_point(expected_meas, map.obstacles, 1);
z_hat = expected_meas.dists;

% Here you need to compute the unnormalized log likelihood of the
% measurement for the given particle.
%
%  Log weight = \Sum_{r=1}^R -((z(r) - \hat{z}(r)).^ 2/(2*sigma2));
sigma2 = 50; % the variance sigma

log_weight = NaN; % <-- dummy value (change this!)
% ----------------------
%  YOUR CODE GOES HERE! 
% ----------------------
log_weight = -sum((z-z_hat).^2)/(2*sigma2^2);
end
