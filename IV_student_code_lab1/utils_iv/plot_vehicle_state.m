function plot_vehicle_state(s, varargin)
    % rotation, translation
    ct = cos(s.theta);
    st = sin(s.theta);
    R = [ct, st; -st, ct];
    T = [s.x; s.y];
    
    slong = 4.5; % longitudinal size (meter)
    slat = 2; % lateral size (meter)
    
    % car outset
    box_in_frame(0, 0, slat, slong, R, T, 'Color', 'k', varargin{:});    
    
    % front windshield
    box_in_frame(0, slong*.05, slat*.8, slong*.2, R, T, 'Color', 'k', varargin{:});
    
    % back window
    box_in_frame(0, slong*-.25, slat*.8, slong*.15, R, T, 'Color', 'k', varargin{:});
    
    % wheel angle
    kappa_mult = 1; % DEBUG: exagarate wheel angle for visualization
    kct = cos(s.kappa * kappa_mult);
    kst = sin(s.kappa * kappa_mult);
    kR = [kct, kst; -kst, kct];
    points = [0, 0 ; [-.2, .2] .* slong];
    points_left = bsxfun(@plus, kR * points, [-.35 * slat; +.3 * slong]);
    points_right = bsxfun(@plus, kR * points, [+.35 * slat; +.3 * slong]);
    plot_in_frame([points_left [NaN;NaN] points_right], R, T, 'Color', 'r', 'LineWidth', 2, varargin{:})

end

function box_in_frame(cx, cy, w, h, R, T, varargin)
    % car outset
    points = [ ...
         1 -1 -1  1  1; ...
        -1 -1  1  1 -1 ...
    ];
    points(1,:) = points(1,:) * w/2. + cx;
    points(2,:) = points(2,:) * h/2. + cy;
    plot_in_frame(points, R, T, varargin{:});
end

function plot_in_frame(points, R, T, varargin)
    % apply transformation
    points = R * points;    
    plot(points(1,:) + T(1), points(2,:) + T(2), 'Tag', 'vehicle', varargin{:});
end
