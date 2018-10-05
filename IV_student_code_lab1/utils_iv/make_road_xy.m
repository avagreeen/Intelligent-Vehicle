function [rx, ry, rtheta] = make_road_xy(rradius, rlong, rlat)
    if nargin < 3; rlat = 0; end
    
    rlat = rlat(:);
    
    if isinf(rradius)
        % special case
        L = numel(rlong);
        D = numel(rlat);
        rtheta = zeros(1,L);
        rx = rlat * ones(1, L);
        ry = ones(D, 1) * rlong;
        return
    end
    
    % compute road center line
    rtheta = rlong / rradius;
    rx = (-rradius + rlat) * cos(-rtheta);
    ry = (-rradius + rlat) * sin(-rtheta);
    
    % let (0,0) not be center of turn, but start point of curve
    rx = rx + rradius;
end