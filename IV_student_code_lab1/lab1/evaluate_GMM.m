% Evaluates GMM with Bayes' rule on test dataset. 
% Input : 
%  - gmm_classifier : struct obtained with train_GMM
%  - test_features  : N x M matrix with N test samples
%
% Output:
%   - decision_vals : Nx1 vector with class posterior
%                     probabilties, i
%                     decision_vals(i) = probability i-th sample is pedestrian 
%
function [decision_vals] =  evaluate_GMM(gmm_classifier, test_features)

[N, M] = size(test_features);
% decision value is the posterior class probability, i.e.
%   decision_vals(i) = probability i-th sample is a pedestrian

decision_vals = zeros(N,1);

% this could be rewritten using Matlab matrix expressions
% it is a for loop for readability

for i = 1:size(test_features,1)
 
    % Bayes' rule to compute posterior of pedestrian given the features:
    
    %                              P(ped) * p(feature | ped)
    % P(ped|feature) = ----------------------------------------------------------
    %                  P(ped) * p(feature | ped)   +   P(garb) * p(feature | garb)
    
    % Get class-conditional pdf from GMMs p(feature | class)
    % See 'doc gmdistribution' to lookup how to evaluate the probability
    % density function of your trained pedestrian/garbage GMMs.
    cond_ped = []; % <-- this should p(feature | ped)
    cond_garb = []; % <-- this should p(feature | garb)
    % ----------------------
    %  YOUR CODE GOES HERE! 
    % ----------------------

    
    % Set uniform class priors
    prior_ped = 0.5; % P(class = ped)
    prior_garb = 0.5; % P(class = garb)
    
    % Use Bayes' rule here to compute decision_vals(i) = P(ped|feature)
    %  from the given cond_ped, cond_garb, prior_ped, prior_garb
    % ----------------------
    %  YOUR CODE GOES HERE! 
    % ----------------------


end


