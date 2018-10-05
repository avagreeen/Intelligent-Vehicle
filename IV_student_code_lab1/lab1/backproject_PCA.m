function features = backproject_PCA(pca, weights)
% Backproject from the D dimensional PCA space to the original M-dimensional feature space.
%
% Input:
% - pca: struct returned by compute_PCA
% - weights: N x D matrix of N samples in the PCA space
%
% Output:
% - features: N x M matrix with the N backprojected samples
%
% SEE ALSO apply_PCA, compute_PCA


% ----------------------

features=weights*pca.components';
% ----------------------


end