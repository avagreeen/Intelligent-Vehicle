function track = make_tracker(m_init, S_init, id, t)
    if nargin < 3; id = 1; end;
    if nargin < 4; t = 1; end;
    
    track.id = id;
    track.t = t;
    track.m = m_init;
    track.S = S_init;

    % Linear Dynamics
    % x_ = Ax + B + N(0, Q)
    %  y = Cx + N(0, R)
    qs = 1e-3; % spatial process noise
    qv = 1e-5; % velocity process noise
    r = 1;
    track.A = [1 0 1 0; 0 1 0 1; 0 0 1 0; 0 0 0 1];
    track.B = [0 0 0 0]';
    track.Q = diag([qs qs qv qv]);
    track.C = [1 0 0 0; 0 1 0 0];
    track.R = diag([r r]);
    
    % historical statistics
    track.ms = track.m;
    track.Ss = track.S;
    track.ts = track.t;
        
    % help functions
    track.predict = @predict;
    track.update = @update;
    track.predict_obs = @predict_obs;
    track.obs_loglik = @obs_loglik;
end

function track = predict(track)
    A = track.A; B = track.B; Q = track.Q;
    track.t = track.t + 1;
    
    track.m = A*track.m + B;
    track.S = A*track.S*A' + Q;
    
    % store result over time
    track.ms = cat(2, track.ms, track.m);
    track.Ss = cat(3, track.Ss, track.S);
    track.ts = cat(2, track.ts, track.t);
end

function [om, oS] = predict_obs(track)
    C = track.C; R = track.R;
    
    om = C*track.m;
    oS = C*track.S*C' + R;
end

function ll = obs_loglik(track, obs)
    [om, oS] = predict_obs(track);
    ll = log(mvnpdf(obs', om', oS'))';
end

function track = update(track, obs)
    C = track.C;
    iS = inv(track.S);
    iR = inv(track.R); 
    
    % FIXME
    track.m = (iS + C' * iR * C) \ (iS * track.m + C' * iR * obs);
    track.S = inv(iS + C' * iR * C);

    % store result over time
    track.ms(:,end) = track.m;
    track.Ss(:,:,end) = track.S;
end
