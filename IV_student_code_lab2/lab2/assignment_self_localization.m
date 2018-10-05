% --------------------------------------------------------
% Intelligent Vehicles Lab Assignment
% --------------------------------------------------------
% Julian Kooij, Delft University of Technology

% clear the workspace
clear all;
close all;
clc;

% setup paths
startup_iv

%% Exercise 2.1: load and visualize the scenario
%-- define occupancy map, vehicle measurements --
jk_srand(1);
scenario = selfloc_scenario(0);

% number of timesteps
T = scenario.T;

% time difference between timesteps (in seconds)
dt = scenario.dt;

map = scenario.map; % occupancy map description of static environment
sensors = scenario.sensors; % the sensor struct for all T timesteps
measurements = scenario.measurements; % the measurement struct for all T timesteps
vehicles = scenario.vehicles; % true position, pf CANNOT use this information
control_inputs = scenario.control_inputs; % 2xT matrix, each column is contol input at time t

% initial true vehicle position
veh_init = vehicles(1);

% -- visualize --
figure(1);
clf;
%plot_setup_selfloc(map);

for t = 1:T
    %%
    veh = vehicles(t);
    sensor = sensors(t);
    meas = measurements(t);

    % plot visualized elements from previous time steps
    delete(findobj('Tag', 'sensor'));
    delete(findobj('Tag', 'vehicle'));

    % plot sensor rays
    %plot_sensor_rays(sensor);
    
    % plot the vehicle state
    %plot_vehicle_state(veh);

    % plot measurements
    %plot_measurements(t, sensor, measurements, 'Marker', '*', 'Color', 'r');

    pause(.001)
end
%% Exercise 2.2: Non-linear Motion model
% You will need to complete the code in
%     pf_predict_step

% initialize particle on initial vehicle position
particle = [veh_init.x; veh_init.y; veh_init.theta];

% duplicate N times
particles = repmat(particle, 1, 1000);

% we will assume for now that the control input remains the same
% during the following 20 predict steps.
velocity = 1; % 1 m/s % <-- you can change this, of course
steering_angle = 0; % (in radians) 0 = go straight % <-- you can change this, of course
control_input = [velocity, steering_angle];

% draw setup
figure(1);
clf;
plot_setup_selfloc(map);
plot_vehicle_state(veh_init);

for step = 1:20
    % perform particle predict step, using a fixed control input
    particles = pf_predict_step(particles, control_input, dt);

    % update plot
    delete(findobj('Tag', 'particles'));
    plot(particles(1,:), particles(2,:), 'm.', 'MarkerSize', 6, 'Tag', 'particles');
    title(sprintf('predicting %d time steps ahead', step))
    
    pause(0.05);
end

%% Exercise 2.3 & 2.4: Measurement likelihood
%
% For Exercise 2.4, you will need to complete the code in
%     map_measurement_loglik

% let's take the first time step as reference
sensor = sensors(1);
vehicle = vehicles(1);
control_input = control_inputs(1);
measurement = measurements(1);

% -- select a test location --
% In a particle filter, particles represent 'candidate' positions, or
% a set of 'hypotheses' about the true state.
% Here we take a look at 7 hypotheses, without considering any vehicle
% dynamics.
test_location_id = 5; % <-- *CHANGE THIS*, try out locations 1 to 7

test_particles = [ ...
        [0; 10; 2*pi * 0/8], ... % first test location
        [0; 10; 2*pi * 2/8], ... % second test location
        [0; 16; 2*pi * 4/8], ... % third test location
        [0; 33; 2*pi * 4/8], ... % etc ...
        [10; 22; 2*pi * 2/8], ...
        [-12; 22; 2*pi * 12/8], ...
        [-10; 22; 2*pi * 13/8] ...
];
particle = test_particles(:, test_location_id);

% The following lines create a hypothetical 'ideal' or 'expected' 
%   measurement for the selected particle.
%   This represents what we *expect* to measure if the vehicle is actually
%   at the particle position.
particle_sensor = copy_sensor(sensor, particle(1), particle(2), particle(3), 0); % copy our sensor, but put it on the particle's state
particle_meas = particle_sensor.new_meas(); % create a new virtual measurement for the sensor
particle_meas = particle_sensor.observe_point(particle_meas, map.obstacles, 1); % measure the obstacles in the map

log_weight = map_measurement_loglik(particle, map, measurement, sensor);
if isnan(log_weight)
    warning('You did not implement map_measurement_loglik correctly yet!');
end
fprintf('expected measurement at particle x_t\nlog weight = %.3f, i.e. weight = %.3f\n', log_weight, exp(log_weight));

% -- visualize --
% setup plot
sfigure(1);
clf;
subplot(1,2,1)
plot_setup_selfloc(map);

% true vehicle state and measurements
plot_vehicle_state(vehicle);
plot_sensor_rays(sensor);
plot_current_measurements(sensor, measurement, 'Marker', '*', 'Color', 'r');

% particle's vehicle state and measurements
plot_vehicle_state(struct('x', particle(1), 'y', particle(2), 'theta', particle(3), 'kappa', 0), 'Color', 'b');
plot_sensor_rays(particle_sensor);
plot_current_measurements(particle_sensor, particle_meas, 'Marker', '*', 'Color', 'b');

% show the ACTUAL sensor measurement
hax1 = subplot(2,2,2);
plot(sensor.angles, measurement.dists, 'r')
grid on
axis tight
ylim([0, 30])
ylabel('distance (m)')
title('actual measurement z_t')

% also show the EXPECTED measurement at the particle's position,orientation
hax2 = subplot(2,2,4);
plot(particle_meas.dists, 'b')
grid on
axis tight
ylim([0, 30])
xlabel('sensor ray')
ylabel('distance (m)')
title('expected measurement at particle x_t')

%% Exercise 2.5, 2.6, 2.7 & 2.8: Particle Filtering
% For Exercise 2.5, you will need to complete the code in
%     pf_update_step

N = 100; % num particles % <-- change this
INITIAL_POSITION_KNOWN = false; % <-- ** Exercise 2.7 **

% compute number of particles to reinitialize each step
frac_reinit = 0.5; % <-- ** Exercise 2.8 ** set fraction here
N_reinit = ceil(N * frac_reinit); % number of particles to reinitialize

% setup plot
figure(1);
clf;
plot_setup_selfloc(map);

% -- initialize particle filter --
if INITIAL_POSITION_KNOWN
    % initial position known
    fprintf('** informed initialization **\n');
    particles = pf_init_around_state(N, veh_init.x, veh_init.y, veh_init.theta); 
else
    % init random on freespace
    fprintf('** random initialization **\n');
    particles = pf_init_freespace(N, map);
end

% -- run particle filtering --
pf = struct;
for t = 1:T
    meas = measurements(t);
    veh = vehicles(t);
    control_input = control_inputs(:,t);
    show t T

    %% Predict step: predict motion and add noise
    particles = pf_predict_step(particles, control_input, dt);

    % Exercise 2.8: randomly reinitialize N_reinit particles
    %   Tip: use pf_init_freespace here
    % ----------------------
    %  YOUR CODE GOES HERE! 
    % ----------------------


    % show particles
    delete(findobj('Tag', 'particles_pred'));
    plot(particles(1,:), particles(2,:), 'm.', 'MarkerSize', 16, 'Tag', 'particles_pred')

    %% Update step: evaluate particle likelihood and resample
    particles = pf_update_step(particles, meas, map, sensor);
    
    %% particle statistics
    m_pos = mean(particles(1:2,:), 2);
    S_pos = cov(particles(1:2,:)');

    %% store
    pf(t).particles = particles;
    pf(t).mean = m_pos;
    pf(t).cov = S_pos;

    %% plot for time t    
    % remove from figure any previously plotted vehicles, particles
    delete(findobj('Tag', 'sensor'));
    delete(findobj('Tag', 'meas'));
    delete(findobj('Tag', 'vehicle'));
    delete(findobj('Tag', 'particles'));
    
    % show particles
    plot(particles(1,:), particles(2,:), 'g.', 'MarkerSize', 5, 'Tag', 'particles')
    
    % show particle statistics with 2D Gaussian distribution
    plot_gauss2d(m_pos, S_pos, 'b', 'LineWidth', 2, 'Tag', 'particles')

    % show actual the vehicle
    plot_vehicle_state(veh);
    
    pause(0.01)
end

%% animate stored result
figure(1)
clf;
plot_setup_selfloc(map);

for t = 1:T
    meas = measurements(t);
    veh = vehicles(t);
    particles = pf(t).particles;
    m_pos = pf(t).mean;
    S_pos = pf(t).cov;
    
    %% update the plot for time t    

    % remove from figure any previously plotted vehicles, particles
    delete(findobj('Tag', 'sensor'));
    delete(findobj('Tag', 'meas'));
    delete(findobj('Tag', 'vehicle'));
    delete(findobj('Tag', 'particles'));
    
    % show particles
    plot(particles(1,:), particles(2,:), 'g.', 'Tag', 'particles')
    
    % show particle statistics with 2D Gaussian distribution
    plot_gauss2d(m_pos, S_pos, 'b', 'LineWidth', 2, 'Tag', 'particles')

    % show actual the vehicle
    plot_vehicle_state(veh);
    
    pause(0.05)
end
