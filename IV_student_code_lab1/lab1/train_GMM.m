% Fit GMM models on both classes in the training data
% 
% Usage:
%   gmm = train_GMM(labels, features)
%   gmm = train_GMM(labels, features, 10) % set regularization C = 10
%
% Input:
%   - labels   : N x 1 vector with label (+1 / -1) of the N samples
%   - features : N x M matrix with features of the N samples,
%                      each row is an M-dimensional feature vector
%   - K        : number of Gaussian mixture components (per class)
%
% Output: 
%   - gmm : a struct containing two fields
%     - gmm_ped : a Gaussian Mixture distribution for the positive
%                 (i.e. pedestrian class)
%     - gmm_garb: a Gaussian Mixture distribution for the negative
%                 (i.e. non-pedestrian, garbage class)
function gmm_classifier = train_GMM(labels, features, K)

% seperate pedestrian and non-pedestrian data
ped_features = features(labels == 1,:);
garb_features = features(labels ~= 1,:);

% fit GMM for pedestrian and non-pedestrian data separately
disp('Fitting GMMs ...');

% fit pedestrian data
%  NOTE: the fit_GMM_single_class subfunction is defined below
gmm_ped = fit_GMM_single_class(ped_features, K);

% fit non-pedestrian data
gmm_garb = fit_GMM_single_class(garb_features, K);

disp('done');

gmm_classifier = struct;
gmm_classifier.gmm_ped = gmm_ped;
gmm_classifier.gmm_garb = gmm_garb;

end

% Fit GMM model to data.
% Fitting a mixture distribution on data is an iterative process, with
%   a random starting point. It may happen that the optimization
%   FAILS to find unstable solutions. 
%   Therefore, the optimization might need to be repeated a few times.
%   However, if such re-initilizations occur a lot, it is probably
%   indicative that you do not have sufficient data for the number of
%   mixutre components.
%
% Look up in the Matlab documentation on how to use the built-in 
%   methods to fit a Gaussian mixture distribution, 
%   e.g. "doc gmdistribution".
%   Find a method to estimate Gaussian mixture parameters with the EM
%   algorithm. The result if this method should be a GMM object
%   that you can later use in evaluate_GMM.
%
% NOTE:
% - Set the maximum number of EM iterations to 500.
%
% - You need to deal with cases where fitting the distribution fails.
%   This means that you need to deal with the situation where the fitting 
%   function throws an error, probably stating:
%
%       Error using ...
%       Ill-conditioned covariance created at iteration x
%
%   Use a TRY-CATCH block to detect these situations, see
%   "doc try" for more information on the syntax.
%   If you catch an error, repeat the the fitting process up to 20 times 
%   until it does succeed. If after those trials it still does not fit,
%   throws your own error with a useful message. See "doc error".
%
% In pseudo-code your function should do the following:
% 
%   FUNCTION fit_GMM_single_class(data, num_components):
%       FOR trial = 1 ... 20:
%           TRY:
%               gmm = fit-gmm(data, num_components, max_iterations=500)
%
%               % Fitting succeeded :)
%               RETURN gmm
%
%           CATCH 'Ill-conditioned covariance' error:
%               % Oops, failed! Print some info to inform the user?
%
%           END TRY
%        END FOR
%
%        THROW ERROR "Could not fit GMM in 20 trials"
%
%   END FUNCTION
%
function gmm = fit_GMM_single_class(data, num_components)

OPTIONS = statset('MaxIter',50,'Display','final','TolFun',1e-6)
for trail=1:20;
	%try
		gmm=fitgmdist(data,num_components,'Options',OPTIONS);

		%return gmm;
	%catch 
        %error('Insufficient data for the number of mixutre components.');
    %end    % ----------------------
end    
    
end