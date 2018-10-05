function show(varargin)
    % SHOW similar to fprintf([cmd '\n'] eval(cmd));
    %    show(param)
    %
    % AUTHOR: Julian Kooij
    % CREATED: 2012-03-01 19:50:00
    %

    single = {}; % single line
    multi = {}; % multi line
    
    for v = 1:nargin
        cmd = varargin{v};
        
        try
            res = evalin('caller', cmd);
        catch
            res = '(undefined)';
        end

        try
            res = mat2str(res, 4);
            single{end+1} = [cmd ' = ' res];
        catch
            res = evalc('disp(res)');
            multi{end+1} = sprintf('%% ---- %s ---- \n%s\n', cmd, res);
        end
    end
    
    % construct output
    S = numel(single);
    for s = 1:S
        fmt = '%s';
        if s > 1; fmt = [' \t' fmt]; end;
        if s == S; fmt = [fmt '\n']; end;
        fprintf(fmt, single{s});
    end
    for m = 1:numel(multi)
        fprintf('%s', multi{m});
    end    
end