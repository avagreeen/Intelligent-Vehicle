function sensor = make_sensor(angle_min, angle_max, M, max_range, dist_sigma, detect_prob, offsets)
if ~exist('max_range', 'var'); max_range = 15; end
if ~exist('dist_sigma', 'var'); dist_sigma = 1; end
if ~exist('detect_prob', 'var'); detect_prob = .5; end
if ~exist('offsets', 'var'); offsets = []; end

sensor.angles = linspace(angle_min, angle_max, M+1);
sensor.angles = sensor.angles(1:end-1);

sensor.max_range = max_range;
sensor.dist_sigma = dist_sigma;
sensor.detect_prob = detect_prob;

sensor.ray_center = [0; 0];
if ~isempty(offsets)
    sensor.angles = sensor.angles -offsets.theta;
    sensor.ray_center = [offsets.x; offsets.y];
end

% ray direction in world coordinates
sensor.ray_dirs = [
    cos(sensor.angles);
    sin(sensor.angles)
];

% add helper functions
sensor.new_meas = @new_meas;
sensor.observe_false_positives = @observe_false_positives;
sensor.observe_point = @observe_point;
sensor.dist_to_pos = @dist_to_pos;

% -- utility functions --
function meas = new_meas()
    meas = struct;
    meas.angles = sensor.angles;
    meas.dists = ones(1,M) * sensor.max_range;
    meas.idxs = nan(1,M);
end

function meas = observe_false_positives(meas, prob, idx)
    if nargin < 3; idx = 0; end
    
    mask = rand(1,M) < prob;
    meas.dists(mask) = rand(1,sum(mask)) * sensor.max_range;
    meas.idxs(mask) = idx;
end

function meas = observe_point(meas, x, radius, idx)
    if nargin < 4; idx = 1; end
    % put sensor center at origin
    x = bsxfun(@minus, x, sensor.ray_center);

    radius2 = radius * radius; % precompute squared radius
    
    % check observations from closest to furthest
    dists2 = sum(x .* x, 1); % squared euclidean norm
    [dists2, order] = sort(dists2);
    x = x(:, order);
    
    % test all rays
    for m = 1:M;
        % distance to ray
        dist_om = sensor.ray_dirs(:,m)' * x;
        dist_om = min(max(dist_om, 0), sensor.max_range);
        proj_om = sensor.ray_dirs(:,m) * dist_om;
        perp_dist2_om = sum((x - proj_om) .* (x - proj_om), 1); % perpendicular squared distance to ray

        % test if object is detected
        prob = (perp_dist2_om < radius2) .* sensor.detect_prob;
        for k = find(rand < prob)
            % noise measurement
            new_dist = randn * sensor.dist_sigma + sqrt(dists2(k));
            if new_dist < meas.dists(m)
                meas.dists(m) = new_dist;
                meas.idxs(m) = idx;
                
                % no need to check other observations
                break;
            end
            
        end
    end
end

function xy_pos = dist_to_pos(dists)
    xy_pos = bsxfun(@times, sensor.ray_dirs, dists);
    xy_pos = bsxfun(@plus, xy_pos, sensor.ray_center);
end

end