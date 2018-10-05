% Compute a D-dimensional PCA space from N samples
%
% pca = compute_PCA(data)
% pca = compute_PCA(data, D)
%
% Input:
% - data : an N x M matrix containing N samples in an M dimensional feature space
% - D: the number of PCA components to retain (optional)
%
% Output: 
% - pca  : struct with the following fields
%   - components : MxD matrix containing the M-d basis vectors of the D PCA
%            dimensions.
%   - mean       : 1xM mean feature vector of the input data
%   - variance   : 1xD vector
%                d-th entry is a number between 0 and 1 that tells the 
%                fraction of variance maintained in the d-th PCA dimension 
%
% SEE ALSO apply_PCA, backproject_PCA

function pca_model = compute_PCA(data, D)

[N, M] = size(data);

% Define default argument (use 'HELP NARGIN' if you do not understand this)
% By default, retain all dimensions in the provided data
if nargin < 2; D = M; end %nargin Number of function input arguments.

pca_model = struct;
pca_model.components = [];
pca_model.mean = [];
pca_model.variance = [];

% ----------------------
%  YOUR CODE GOES HERE! 
 %calculate the mean of each feature
[princa, score, eignvalue]=pca(data);
pca_model.components=princa(:,[1:D]);
pca_model.mean = mean(data);

pca_model.variance=eignvalue([1:D])'/sum(eignvalue);
% check that everything is OK
assert(all(size(pca_model.components) == [M, D]));
assert(all(size(pca_model.mean) == [1, M]));
assert(all(size(pca_model.variance) == [1, D]));
end