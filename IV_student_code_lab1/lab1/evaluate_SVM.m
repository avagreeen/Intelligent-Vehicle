% Evaluates linear SVM on dataset, returns decision value
%
% Input: 
%   - svm           : linear SVM struct from train_SVM
%   - test_features : NxM matrix with N test samples
%
% Output:
%   - decision_vals : Nx1 vector with classifier decision values                    
%
function [decision_vals] =  evaluate_SVM(svm, test_features)

[N, M] = size(test_features);

% let x_i be the Mx1 column feature vector of sample i,
%   and w be the M dimensional Linear SVM weights,
%   and b be the Linear SVM bias,
% then the SVM decision value d_i for this sample is simply
%   d_i = w^T * x_i + b
%
% The sign of this decision value can be used as the classification label

 decision_vals = NaN(N, 1);% dummy values

% ----------------------
d_values=svm.w'*test_features'+svm.b;
decision_vals=d_values';

% ----------------------


% check all ok
assert(all(size(decision_vals) == [N, 1]));
