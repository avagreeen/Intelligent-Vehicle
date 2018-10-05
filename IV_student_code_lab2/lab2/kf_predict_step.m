function kf = kf_predict_step(kf)

% get the last (i.e. updated) state from the previous time step
mu_prev = kf.mu_upds(:,end);
Sigma_prev = kf.Sigma_upds(:,:,end);
t = kf.ts(end);

% Write here the two Kalman predict equations here to compute
%   from previous state distribution of t-1, N(x_{t-1} |mu_prev, Sigma_prev)
%   the predicted state distribution for t, N(x_t | mu, Sigma)
% ----------------------
%  YOUR CODE GOES HERE! 
% ----------------------
mu=kf.F*mu_prev;
Sigma=kf.F*Sigma_prev*kf.F'+kf.Sigma_x;



% after the predicted mu and Sigma have been computed,
%   store them in the struct as the latest predicted state
kf.mu_preds(:,end+1) = mu;
kf.Sigma_preds(:,:,end+1) = Sigma;
% we also set the 'updated' values for the new time step
% equal to the predict ones 
kf.mu_upds(:,end+1) = mu;
kf.Sigma_upds(:,:,end+1) = Sigma;
kf.ts(end+1) = t + 1;

end