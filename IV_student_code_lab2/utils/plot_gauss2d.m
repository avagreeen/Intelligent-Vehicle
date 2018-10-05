function hs = plot_gauss2d(mu, Sigma, varargin)
    hs = [];
    radii = 1;
    if numel(varargin) > 0 && isnumeric(varargin{1})
        radii = varargin{1};
        varargin = varargin(2:end);
    end

    try
        % NOTE: chol might fail if Sigma is not positive-definite
        C = chol(Sigma);
        L = linspace(0., 2*pi, 100);
        L = [cos(L); sin(L)];
        
        for radius = radii
            X = C' * (L .* radius);

            hs(end+1) = plot(X(1,:) + mu(1), X(2,:) + mu(2), varargin{:});
        end
    catch
        % oops, cannot reliably compute covariance
        % o well
    end
    
    if nargout == 0
        clear hs
    end
end