function animate_kf_uncertainty_regions(kf, dims)
    
    assert(numel(dims) == 2);
    assert(all(ismember(dims, 1:4)));
    dim_names = {'state dim 1 - x', 'state dim 2 - y', 'state dim 3 - vel. x', 'state dim 4 - vel. y'};

    % setup figure
    figure(3);
    clf
    axis equal
    grid on;
    axis([-1 1 -1 1] * 4);
    hold all
    xlabel(dim_names{dims(1)});
    ylabel(dim_names{dims(2)});

    colors = jet(numel(kf.ts));
    for step = 1:numel(kf.ts);
        t = kf.ts(step);

        % get the updated
        mu = kf.mu_upds(:, step);
        Sigma = kf.Sigma_upds(:, :, step);

        % plot the uncertainty region of the two given dimensions
        display_name = sprintf('time step %d', t);
        plot_gauss2d(mu(dims), Sigma(dims,dims), 'Color', colors(step,:), 'DisplayName', display_name);
        title(sprintf('KF state distribution @ time step %d', t))

        % update figure
        pause(.1)
    end
    
    legend_by_displayname
    title('KF state distribution')
end