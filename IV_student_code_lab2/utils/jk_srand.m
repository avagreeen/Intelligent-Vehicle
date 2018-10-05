function rand_state = jk_srand(seed)
    % Get/Set random seed
    % SEE ALSO rand casbnet_train
    
    if (nargin > 0) && ~isempty(seed)
        if isnumeric(seed) && numel(seed) == 1
            % old-style random seed

            rand('seed', seed);
            randn('seed', seed);

        elseif isnumeric(seed)
            % set state of MATLAB random state generator
            % (this syntax only works from version >= 7.7)
            % SEE ALSO rand()
            
            %defaultStream = RandStream.getDefaultStream; % DEPRICATED
            defaultStream = RandStream.getGlobalStream;            
            defaultStream.State = seed;

        elseif iscell(seed)
            % set state of MATLAB random state generator
            % (this syntax only works from version >= 7.7)
            % SEE ALSO rand()
            
            %defaultStream = RandStream.getDefaultStream; % DEPRICATED
            defaultStream = RandStream.getGlobalStream;
            defaultStream.State = seed;
            
        else
            error('incorrect value for option Seed\n');
        end
    end
    
    if nargout > 0
        % get state of MATLAB random state generator
        % (this syntax only works from version >= 7.7)
        % SEE ALSO rand()
        
        %defaultStream = RandStream.getDefaultStream; % DEPRICATED
        defaultStream = RandStream.getGlobalStream;
        
        rand_state = defaultStream.State;
    end
    
end