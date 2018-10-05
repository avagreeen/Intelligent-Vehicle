function measurements = preprocess_measurements(measurements)

filter_order = 3; % <-- ** Exercise 1.10 ** order of the median filter

T = numel(measurements);

% iterate over all time steps
for t = 1:T
    dists = measurements(t).dists;
    
    % remove noise from the M-dimensional dists vector
    dists = medfilt1(dists, filter_order);
    
    measurements(t).dists = dists;
end

end