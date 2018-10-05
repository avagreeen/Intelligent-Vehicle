function hl = legend_by_displayname(varargin)
    % LEGEND_BY_DISPLAYNAME Legend of only plots that have DisplayName set
    %   hl = legend_by_displayname(varargin)
    %
    % EXAMPLE:
    %    clf; hold all;
    %    plot(xs, ys1, '-', 'DisplayName', 'readings') % included in legend
    %    plot(xs, ys2, '--'); % not included
    %    legend_by_displayname;
    %
    %
    % AUTHOR: Julian Kooij
    % CREATED: 2012-03-29 18:49:35
    %

    % get handles
    %hs = findobj(gca, 'Type', 'line', '-not', 'DisplayName', '');
    hs = findobj(gca, '-property', 'DisplayName', '-not', 'DisplayName', '');
    if isnumeric(hs)
        hs = sort(hs); % sort by order of creation
    end
    % put every unique DisplayName only once in the legend
    names = arrayfun(@(h) get(h, 'DisplayName'), hs, 'UniformOutput', 0);
    
    % remove double entries
    [~, idxs] = unique(names, 'first');
    idxs = sort(idxs);
    hs = hs(idxs);
    hs = hs(end:-1:1);
    
    % create the legend
    hl = legend(hs);
    
    if isprop(hl, 'AutoUpdate')
        % New autoupdate feature introduced in Matlab R2017a
        % introduces side effects in existing animations using
        % legend_by_displayname, and slows down plot updating.
        % Turn it off ...
        hl.AutoUpdate = 'off';
    end
    
    if numel(varargin) > 0
        set(hl, varargin{:});
    end
    % varargin options for legend are set in a seperate 'set' call
    % because I had problems with ('interpreter', 'none').
    
    if nargout == 0
        clear hl;
    end
end
