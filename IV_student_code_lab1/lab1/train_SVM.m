% Train a linear SVM on given training data
% 
% Usage:
%   svm = train_SVM(labels, features)
%   svm = train_SVM(labels, features, 10) % set regularization C = 10
%
% Input:
%   - labels   : N x 1 vector with label (+1 / -1) of the N samples
%   - features : N x M matrix with features of the N samples,
%                      each row is an M-dimensional feature vector
%   - C        : regularization parameter (i.e. sensitivity to outliers)
% Output: 
%   - svm : a struct containing two fields
%     - w - linear SVM weight vector
%     - b - linear SVM bias
%

function svm = train_SVM(labels, features, C)
if nargin < 3;
    C = 2; % NOTE: if no C argument is given, we use C = 2 here by default
end;

% training set
%  N x M matrix
[N, M] = size(features);

% training set labels (1 = ped, -1 = garbage)
%  N x 1 vector

% initialize empty return struct containing the linear SVM parameters
%  (these parameters will be set below by training on the data)
svm = struct;
svm.w = []; % <-- this should contain the M-dimensional SVM weights vector
svm.b = []; % <-- this number should be the SVM bias

disp('running svm training ...');

% There are two ways how you can train a Linear SVM in this exercise:
%
% First, you can use primal_svm.m which is provided in the lab
% folder. You can run this, even on older Matlab versions, but it has
% some quirks.
%
% Second option is to use Matlab's builtin SVM classifier. However,
% older Matlab versions might have this tool available.
% 
% If your Matlab installation DOES NOT HAVE fitscm, use the first Option.
%
% ----------------------------------------------------------------
% Option 1: Using primal_svm.m
% ----------------------------------------------------------------
% The provided primal_svm.m is a simple pure Matlab implementation
% to train an SVM classifier using gradient descent.
%
% Use 'help primal_svm' for information on how to use it.
%
% NOTE: you should use the following options:
%  - LINEAR = 1   : use a Linear SVM
%  - LAMBDA = 1/C : note that this function defines its regularization
%                   as the inverse of parameter C
% There is no need to change any of the other default options.
%
% One IMPORTANT consequence of using the LINEAR = 1 option, is that
% primal_svm looks for the data as a GLOBAL variable X.
% So before calling primal_svm, you must define "GLOBAL X;" in this
% this function, and assign to X the training data matrix.
% See "help global".
%
% primal_svm will return the linear weights and bias directly, see its
% help text.
%
% ----------------------------------------------------------------
% Option 2: Using Matlab's builtin SVM classifer fitcsvm
% ----------------------------------------------------------------
%
% NOTE: we want to set the following properties
%   (use 'doc fitcsvm' to lookup how to set them) :
%
% - we do NOT want fitcsvm to standardize our training data, disable it
% - we want to use C as the 'BoxConstraint'
% - set the iteration limit (= max number of iterations) to 100000
% - (optional) use can set the 'Verbose' option to 1
%
% ------------------------------------------------------
% Read this carefully
% ------------------------------------------------------
% After you succesfully invoked fitcsvm, it will return a trained SVM
% object of the class `ClassificationSVM'.
% At a later stage, the built-in `predict' function could be
% used with this trained object to classify new samples.
%
% In this exercise, we will NOT use the built-in `predict' for didactic
% purposes, and because it is a bit slower (for our simple use case).
%
% Therefore, you should NOT return the ClassificationSVM model.
% Instead, we want to return ONLY the parameters of the linear 
% decision boundary, and store them in the output struct.
%
% These parameters are:
%  - w : M-dimensional vector with the SVM weights
%  - b : a real valued number with the SVM bias
%
% The parameters can easily be obtained as the 'Beta' and 'Bias' field
% from the ClassificationSVM object.
%
% See: doc ClassificationSVM

% ----------------------
global X;
X=features;

[svm.w svm.b]=primal_svm(1,labels,1/C,[]);
 
% ----------------------


assert(all(size(svm.w) == [M, 1]));
assert(all(size(svm.b) == [1, 1]));




