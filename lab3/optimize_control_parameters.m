function params = optimize_control_parameters(config, goal_state, init_params)

% configure optimzation algorithm
opt_algo_options = optimoptions('lsqnonlin', ...
    'Display', 'iter', ... % DEBUG
    'TolX', 1e-10, ...
    'TolFun', 1e-10, ...
    'MaxFunEvals', 100 ...
);

% run non-linear optimization to find parameter p
%    p = argmin_{p} |C(p)|^2
% using gradient descent. See HELP LSQNONLIN
params = lsqnonlin( ...
    @C, ...
    init_params, [], [], opt_algo_options);

% this is the error function
function err = C(params)
    % This function should return a vector of errors to minimize,
    % given the control parameters.
    %
    %  DO NOT ADD ANY OF ARGUMENTS TO THIS FUNCTION.
    %  IT SHOULD REMAIN C(params) FOR lsqnonlin TO WORK.
    %
    % NOTE that variables `config` and `goal_state` are already
    %  accessible from within this function.
    %
    % The predicted vehicle states can be obtained with 
    %     curv_param_state_predict.
    % The compute the error of the x, y and theta between the
    % final predicted state and the goal state.

    % ----------------------
    %  YOUR CODE GOES HERE! 
    % ----------------------

    
end

end
