function [is_ok, score] = test_gating_score(kf, meas_r)
    % get the expected measurement position from the Kalman Filter
    pred_z_mu = kf_predict_obs(kf);
    
    % the given observation and the 'expected observation' pred_z_mu
    %   should have the same dimensions. Just checking if everything is ok.
    assert(all(size(meas_r) == size(pred_z_mu)))
    
    % For the gating procedure we can use the Euclidean distance.
    %   Let the `score` variable be the distance
    % You also need to define the `gating_threshold` to which the score
    % is compared. 
    % Assuming that all position (and hence Euclidean distances) are given
    % in meters, try out a gating threshold between 1 and 10 meter,
    % and pick one you think works well.
    %
    % Euclidean distance score:
    %                 ________________________________________
    %           _    /  ---
    %   score =  \  /   \     ( meas_r(d) - pred_z_mu(d) )^2
    %             \/    /__ d 
    %
    % The measurement can be accepted iff (score < gating_threshold)
    
    % *NOTE*
    % the gating threshold is some fixed number for YOU (student) to determine
    gating_threshold = 3; 
    
    % this will be the score of the measurement for this KF    
    %   (the score will also be used for data association later on)
    score = NaN;
    
    % ----------------------
    %  YOUR CODE GOES HERE! 
    % ----------------------
    score2=sqrt(sum((meas_r-pred_z_mu).^2));
    

    % compare score to threshold
    is_ok = (score < gating_threshold);
end
