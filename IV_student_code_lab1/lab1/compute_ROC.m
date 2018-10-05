% roc = compute_ROC (labels, decision_vals)
%
% Compute ROC curves.
%
%   LabelSet:       Column vector of n true labels {1,-1}, 1 = target class
%   DecisionValues: Column vector of n classifier decision values
%
%   Output: roc - nx2 matrix : [false pos rate, true pos rate]
%


function roc = compute_ROC(labels, decision_vals)

% labels of target class
TargetLabels = (labels == 1);

if (sum(TargetLabels) == 0)
    error ('no target labels (id 1) found');
end

% Remember that:
%   True Positive Rate (TPR) = # of true positives / # of positives
%   False Positive Rate (FPR)  = # of false positives / # of negatives
%
% To compute the ROC curves, one needs to:
%  1. count the number of positive samples P (i.e. true label is +1)
%  2. count the number of negative samples N (i.e. true label is -1)
%  for the i-th possible decision thresholds
%    3. count the number of true positives,
%        i.e. samples with decision value above the threshold, which
%        also have a true label of +1
%    4. count the number of false positives,
%        i.e. samples with decision value above the threshold, which
%        also have a true label of -1
%    5. compute the TPR and FPR
%    6. set the [FPR, TPR] as the i-th row in the output matrix.


% ----------------------
%  YOUR CODE GOES HERE! 
% ----------------------

