function plot_kf_object_comparison(kf, object)

if (numel(kf) ~= 1)
    warning('expected single Kalman Filter struct, using first one only');
    kf = kf(1);
end
if(numel(object) ~= 1);
    warning('expected single object struct, using first one only');
    object = object(1);
end

T = size(object.pos, 2);

kf_ts = kf.ts;
kf_indices = find(ismember(kf_ts, 1:T));
kf_pos = kf.mu_upds(1:2, :);
object_ts = 1:T;
object_indices = find(ismember(object_ts, kf_ts));
object_pos = object.pos(:, object_indices);

err = kf_pos(:, kf_indices) - object_pos(:, object_indices);
err = sqrt(sum(err .* err, 1));

%% create plot
dim_names = {'x position', 'y position'};

figure(4)
clf
for dim = 1:2
    subplot(3,1,dim)
    
    % plot true and filtered position on x or y dimension
    plot(kf_ts, kf_pos(dim, :), 'DisplayName', 'KF')
    hold all
    plot(object_ts, object_pos(dim, :), 'DisplayName', 'actual position')
    ylabel(dim_names{dim})
    grid on
    legend_by_displayname
end

% plot the (temporally aligned) Eudclidean distance
subplot(3,1,3)
plot(kf_ts(kf_indices), err, 'DisplayName', 'error')
grid on
xlabel('time')
ylabel('error (Euclidean)')

% enable that zooming in one subplot zooms all subplots
linkaxes(findobj(gcf, 'type', 'axes'), 'x')

end