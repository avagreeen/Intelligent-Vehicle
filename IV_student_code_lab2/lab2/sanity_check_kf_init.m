% Perorm some tests to see if kf_init is probably implemented correctly
function sanity_check_kf_init(kf)

% DON'T LOOK IN HERE, THERE IS NOTHING TO SEE HERE!
%   (don't mind the man behind the curtain)
% The true matrices are not in here.
%
% Instead, the sanity check applies a very simple
%   mathematical function to your matrices and 
%   compares it to the expected outcome of the correct 
%   solutions.
% 

is_ok = true;

% do a few sanity checks
fprintf('-- KF_INIT --\n'); 
is_ok = is_ok & check_matrix_size('F', kf.F, [4, 4]);
is_ok = is_ok & check_matrix_size('H', kf.H, [2, 4]);
is_ok = is_ok & check_matrix_size('Sigma_x', kf.Sigma_x, [4, 4]);
is_ok = is_ok & check_matrix_size('Sigma_z', kf.Sigma_z, [2, 2]);
is_ok = is_ok & check_matrix_hash('F', kf.F, 13230);
is_ok = is_ok & check_matrix_hash('H', kf.H, 4274);
is_ok = is_ok & check_matrix_hash('Sigma_x', kf.Sigma_x, 474.08);
is_ok = is_ok & check_matrix_hash('Sigma_z', kf.Sigma_z, 12822);

if ~is_ok
    warning('Oops, kf_init() is not as expected!')
else
    fprintf('All tests OK :-)\n')
end
fprintf('\n');

end

function s = yes_no_string(boolean)
    boolean_strings = {'NO', 'YES'};
    s = boolean_strings{(boolean ~= false)+1};
end

function is_ok = check_matrix_size(name, m, target_size)
    % compare matrix size
    actual_size = size(m);
    is_ok = all(actual_size == target_size);
    
    % format message
    s_target_size = mat2str(target_size);
    s_actual_size = mat2str(actual_size);
    s_is_ok = yes_no_string(is_ok);
    fprintf('size of %s sould be %s, and is %s. check ok? %s\n', name, s_target_size, s_actual_size, s_is_ok)
end

function is_ok = check_matrix_hash(name, m, target_hash)
    % perform poorly implemented hash check ...
    
    % compute hash
    primes = [2131, 2137, 2141, 2143, 2153, 2161, 2179, 2203, 2207, 2213, 2221, 2237, 2239, 2243, 2251, 2267];
    P = numel(primes);
    m = [m(:); zeros(P,1)];
    actual_hash = sum(m(1:P) .* primes(:));

    %show name actual_hash % DEBUG
    
    % compare hash
    is_ok = (abs(target_hash - actual_hash) < 1e-10);
    s_is_ok = yes_no_string(is_ok);    
    fprintf('content of matrix %s ok? %s\n', name, s_is_ok)
end
