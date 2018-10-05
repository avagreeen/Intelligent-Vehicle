% Define the steering profile
% Input:
%   - k0, k1, k2 : the spline control inputs at a fraction of 0, .5 and 1
%                  (i.e. 0%, 50% and 100%) of the traveled distance only
%                  the road segment
% Output:
%   - steering_profile : a *function* which maps sfrac to steering angles
%         if given a 1xN vector sfrac with N fractions between 0 and 1,
%         steering_profile(sfrac) should return a 1xN vector with
%         corresponding steering angles.
%
% Note that the profile function should satisfy the following properties:
%     steering_profile(0) == k0
%     steering_profile(.5) == k1
%     steering_profile(1) == k2
%     steering_profile(0.25) % a value between k0 and k1
%     steering_profile([0 .5]) == [k0 k1]
%
function steering_profile = make_steering_profile(k0, k1, k2)

    % This dummy function shows a dummy example of the steering profile.
    % It just returns same random output independent of the input values
    steering_profile = @(sfrac) rand(size(sfrac))-.5; % dummy function
    
    % Tip: use Matlab's builtin 'spline' function to define an anomous
    % function (similar to the dummy function above)
    % If you have never made an anonymous function in Matlab before, see
    %   https://nl.mathworks.com/help/matlab/matlab_prog/anonymous-functions.html
    % ----------------------
    %  YOUR CODE GOES HERE! 
    % ----------------------

    
    % Check that the function returns a steering angle for
    debug_test_equal(steering_profile, 0, k0);
    debug_test_equal(steering_profile, .5, k1);
    debug_test_equal(steering_profile, 1, k2);
    debug_test_equal(steering_profile, [1 0 .5], [k2 k0 k1]); % should also work out of order
    
end

% Test that two vectors x and y are almost equal
function debug_test_equal(steering_profile, sfrac, true_out)
    out = steering_profile(sfrac);
    if ~all(numel(out) == numel(true_out));
        warning('steering_profile: different number of actual outputs and expected outputs');
        return;
    end
    if ~all(size(out) == size(true_out));
        warning('steering_profile: actual output and expected output have different sizes');
        return;
    end
    if ~all(abs(out - true_out) < 1e-10);
        warning('steering_profile: actual output and expected output differ too much');
        return;
    end
    
    % all, ok
end
