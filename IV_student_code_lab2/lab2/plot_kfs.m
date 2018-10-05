function plot_kfs(t, kfs)
        tracker_color = lines(numel(kfs));

        % remove the currently plot tracks, if any
        delete(findobj('Tag', 'track'));

        % plot tracker results
        for j = 1:numel(kfs)
            %%
            kf = kfs(j);
            if all(kf.ts ~= t); continue; end
            
            % grab the positional mean and covariance
            % of the filtered state up to time t
            %
            %  (note that dimensions 1 & 2 of the state
            %  correspond to the filtered position,
            %  and the dimensions 3 & 4 to the filtered
            %  velocity).
            tmask = (kf.ts <= t) & (kf.ts > 0);
            
            mus = kf.mu_upds(1:2, tmask);
            Sigmas = kf.Sigma_upds(1:2,1:2, tmask);

            color = tracker_color(j, :);

            % plot the filtered mean position up to the current time
            plot(mus(1,:), mus(2,:), '-', 'Color', color, 'Tag', 'track');
            
            % plot the state distribution of the last time step
            %  (i.e. time t)
            plot_gauss2d(mus(1:2,end), Sigmas(1:2,1:2,end), 'Color', color, 'Tag', 'track');

            if 1
                % also plot the observation likelihood
                z_mu = kf.H * kf.mu_upds(:,kf.ts == t);
                z_Sigma = kf.H * kf.Sigma_upds(:,:,kf.ts == t) * kf.H' + kf.Sigma_z;
                plot_gauss2d(z_mu, z_Sigma, '--', 'Color', color, 'Tag', 'track');
            end
        end
end
