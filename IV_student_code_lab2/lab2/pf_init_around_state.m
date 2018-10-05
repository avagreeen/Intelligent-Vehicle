function particles = pf_init_around_state(N, x, y, theta)
    % init at true location
    particles = repmat([x, y, theta]', 1, N);
    
    % add a bit of variation
    particles(1:2,:) = particles(1:2,:) + (rand(2,N)-.5);
    particles(3,:) = particles(3,:) + rand(1, N) * 2*pi / 16;
end