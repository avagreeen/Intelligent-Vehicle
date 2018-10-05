% Particle Filter predict step
% Input:
%   - particles: 3 x N matrix with N particles (as columns) of previous
%     time step
%   - control_input: a 2D vector with [forward velocity, steering angle]
%   - dt: time step duration (in seconds)
% Output:
%   - particles: 3 x N matrix with N updated particles for new time step

function particles = pf_predict_step(particles, control_input, dt)
    % number of particles
    N = size(particles, 2);
    
    % in this simple model there is only velocity and steering
    velocity = control_input(1);        % [   v   ] forward velocity
    steering_angle = control_input(2);  % [ omega ] steering angle

    % Here the prediction happens with the simple motion model,
    %  and the added Gaussian noise
    
    %% deterministic motion
    % First apply deterministic motion model:
    %    [  x_t  ]      [  x_t-1  ]    [ v * dt * sin(theta_t-1) ]
    %    [  y_t  ]  =   [  y_t-1  ] +  [ v * dt * cos(theta_t-1) ]
    %    [theta_t]      [theta_t-1]    [       omega * dt        ]
    
    % ----------------------
    %  YOUR CODE GOES HERE! 
    % ----------------------
    particles(1,:)=particles(1,:)+velocity*dt*sin(particles(3,1));
    particles(2,:)=particles(2,:)+velocity*dt*cos(particles(3,1));
    particles(3,:)=particles(3,:)+steering_angle*dt;

    %% add noise
    % Second, add here some random noise to each particle.
    % You can use mvnrnd to generate noise to add to the particles.
    % For each particle i, sample a noise vector
    %     \eta^{(i)} ~ N(0, Sigma_x).
    % Note that with mvnrnd you can sample all N noise vectors in one go.
    
    Sigma_x = diag([
        .1, ... % std.dev. on x position is 0.2 meters
        .1, ... % std.dev. on y position is 0.2 meters
        (2*pi / 24).^2 ... % std.dev. on angle is 1/24 of full 360 degrees circle
    ]);

    % ----------------------
    %  YOUR CODE GOES HERE! 
    % ----------------------
    %noise=repmat(mvnrnd([0 0 0], Sigma_x)',1,1000);
    %particles=particles+noise;
    % since the dimension of each particle, i.e. orientation theta,
    % is on the circular domain, let's ensure all values remain on the
    % circular [0, 2*pi] domain.
    particles(3,:) = mod(particles(3,:), 2*pi);
end