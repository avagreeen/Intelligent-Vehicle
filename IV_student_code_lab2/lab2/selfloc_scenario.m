function scenario = selfloc_scenario(frac_to_remove)

if nargin < 1; frac_to_remove = 0; end % fraction of walls to remove (optional)

map = define_occupancy_map(frac_to_remove);

% vehicle state
veh_init = struct( 'x', 0, 'y', 10, 'theta', 0e-2 * 2*pi, 'kappa', 0, 'v', 1, 'a', 0);

% a slightly more complicated vehicle model
motion_model = @(s, u, dt) struct( ...
    'x',     s.x + s.v * sin(s.theta) * dt, ...
    'y',     s.y + s.v * cos(s.theta) * dt, ... 
    'theta', s.theta + s.v * s.kappa * dt, ...
    'kappa', u.kappa, ...
    'v',     s.v + s.a * dt, ... % <-- NOTE: different from Algorithm 3
    'a',     u.a ...
);

% control input over T timesteps
T = 90;
dt = .5;
u_as = zeros(1,T);
u_ks = zeros(1,T);
u_ks(20:25) = -pi/6;
u_ks(42:47) = -pi/6;
u_ks(65:70) = +pi/6;
us = struct('a', num2cell(u_as), 'kappa', num2cell(u_ks));

% simulate vehicle motion and observations
veh = veh_init;
control_inputs = NaN(2, T);
for t = 1:T
    veh = motion_model(veh, us(t), dt);

    % create range sensor on vehicle
    M = 16*2;
    sensor = make_sensor(0, 2*pi, M, 25, 5e-1, 1, veh);
    meas = sensor.new_meas();
    meas = sensor.observe_point(meas, map.obstacles, 1);

    vehicles(t) = veh;
    sensors(t) = sensor;
    measurements(t) = meas;
    control_inputs(:,t) = [veh.v; veh.kappa];
end

% put everything in a convenient struct
scenario = struct;
scenario.T = T;
scenario.dt = dt;
scenario.map = map;
scenario.sensors = sensors;
scenario.measurements = measurements;
scenario.vehicles = vehicles;
scenario.control_inputs = control_inputs;

end