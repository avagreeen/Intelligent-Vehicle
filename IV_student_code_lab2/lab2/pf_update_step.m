% Particle Filter update step
%   Evaluates measurement log-likelihood for all particles,
%   and resamples, with probability proportional to their likelihood.
function new_particles = pf_update_step(particles, meas, map, sensor)
    % number of particles
    N = size(particles, 2);

    % this list will contain the (log of the) weights for all N particles
    log_weights = nan(1,N);

    %% evaluate particle likelihood

    % For each particle:
    %     compute the (unnormalized) measurement log-likelihood
    %     store the log-likelihood of the j-th particle as weights(j)
    % Tip 1: use a simple for-loop to iterate over all N particles
    % Tip 2: use map_measurement_loglik

    % ----------------------
    %  YOUR CODE GOES HERE! 
    % ----------------------
    for i=1:N
        log_weights(i)=map_measurement_loglik(particles(:,i),map,meas,sensor);
    end

    %% resample particles 

    % We construct construct normalized probabilities from the list of
    %   log-likelihoods. Basically, we need to convert log-likelihoods to
    %   normal likelihoods, and then divide by their sum to obtain a
    %   normalized probability distribution (i.e. that sums to 1).
    %
    % Some background information on the following lines:
    % Before converting to normal probabilities, we perform a 'trick'
    % to ensure the the unnormalized probabilities do not become too small
    % to suffer from the rounding errors of the computer when using exp().
    % The trick is to multiply the probabilities by some rescaling factor,
    % which in log-space means adding or subtracting the log factor.
    % This scaling factor will be removed in the final normalization
    % step anyway.
    probs = log_weights; % start with the original log-likelihoods
    probs = probs - max(probs(:)); % the "big trick"
    probs = exp(probs); % convert log-probabilities to probabilities
    probs = probs ./ sum(probs); % normalize

    % now probs should sum to one (with minimum tolerance for nasty
    % rounding errors).
    assert(abs(sum(probs) - 1) < 1e-10);

    % Sample a new set of N particles (with replacement) from old set,
    % and call this set new_particles
    %
    % NOTE: you can use the RANDSELECT() function to sample with replacement.
    % For example:
    %    probs = [.4, .5, .1]
    %    idxs = randselect(probs, 100);
    % Then idxs is a 100-dimensional vector with values distributed approximately
    %    40% the value 1, 50% the value 2, 10% the value 3
    % 
    % ----------------------
    %  YOUR CODE GOES HERE! 
    % ----------------------
    idxs=randselect(probs, N);
    new_particles=particles(:,idxs);
    % new_particles should by a 3 x N matrix
    assert(all(size(new_particles) == [3, N])); 
end