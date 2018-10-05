% --------------------------------------------------------
% Intelligent Vehicles Lab Assignment
% --------------------------------------------------------
% Julian Kooij, Delft University of Technology

% clear the workspace
clear all;
close all;
clc;

% setup paths
startup_iv

%% prepare the training and test data

% add / set paths
data_path = fullfile(IV_BASE_PATH, 'lab1_data/');

% load data
disp('Loading data ...');
load(fullfile(data_path, 'data_labels.mat'));
load(fullfile(data_path, 'data_hog.mat'));
load(fullfile(data_path, 'data_lrf.mat'));
load(fullfile(data_path, 'data_int.mat'));
disp('done');

% concatenate pedestrian and non-pedestrian data
train_labels = [ped_train_labels; garb_train_labels];
all_train_hog = [ped_train_hog; garb_train_hog];
all_train_lrf = [ped_train_lrf; garb_train_lrf];
all_train_int = [ped_train_int; garb_train_int];

%% Exercise 1.1: Visualize the intensity data.
% You will need to complete the code in
%    imshow_intensity_features.m

figure(1);
clf;
ncols = 6;
for j = 1:ncols
    % show j-th pedestrian sample
    subplot(2, ncols, j)
    I = ped_train_int(j, :);
    imshow_intensity_features(I);

    % show j-th garbage sample
    subplot(2, ncols, j+ncols)
    I = garb_train_int(j, :);
    imshow_intensity_features(I);
end

%% Exercise 1.2: Apply PCA to the gray-scale intensity features.
% You will need to complete the code in
%    compute_PCA.m

num_PCA_components = 10;

% compute PCA space on the intensity features
pca_int = compute_PCA(all_train_int, num_PCA_components);

% Visualize the mean pedestrian
figure(1); clf;
imshow_intensity_features(pca_int.mean);
title('mean image');

% Visualize the "eigen-pedestrians"
figure(2); clf;
num_rows = 2;
num_cols = 5;
for j = 1:(num_rows*num_cols)
    subplot(num_rows, num_cols, j);

    I = pca_int.components(:,j);
    imshow_intensity_features(I);

    title(sprintf('PCA %d', j));
end

%% Exercise 1.3: Project data to the PCA space.
% Project the training data to the 3-dimensional PCA space, and visualize
% the resulting distribution.
% You will need to complete the code in
%    apply_PCA.m

% compute the PCA space
num_PCA_components = 3;
pca_int = compute_PCA(all_train_int, num_PCA_components);

% project the datva onto the computed PCA space
proj_ped = apply_PCA(pca_int, ped_train_int);
proj_garb = apply_PCA(pca_int, garb_train_int);

% create 3D plot of data in PCA space
figure(3); clf;
plot3(proj_ped(:,1), proj_ped(:,2), proj_ped(:,3), 'g.', 'DisplayName', 'pedestrians');
hold all;
plot3(proj_garb(:,1), proj_garb(:,2), proj_garb(:,3), 'r.', 'DisplayName', 'background');
xlabel('PCA 1')
ylabel('PCA 2')
zlabel('PCA 3')
grid on
axis equal
legend_by_displayname

%% Exercise 1.4: Back-projecting to the feature space
% Project the D-dimensional PCA representation of several pedestrians
% in the training data back to the original feature space.
% Since we are using pixel intensities as features, we can visualize the
% backprojected result.
% You will need to complete the code in
%    backproject_PCA.m

% back-project images from the reduced PCA space
num_PCA_components = 100; % <-- Try out different numbers here !

pca_int = compute_PCA(all_train_int, num_PCA_components);
ids = [1:6] * 400; % select a few samples in the training data
selected_orig = all_train_int(ids,:);

% project images to PCA space
proj_ped = apply_PCA(pca_int, selected_orig);

% now back-project from the D-dimensional PCA space to the original image space
selected_backprojected = backproject_PCA(pca_int, proj_ped);

% compare the results
figure(1);
clf;
ncols = numel(ids);
for j = 1:ncols
    % show j-th original sample
    subplot(2, ncols, j)
    I = selected_orig(j, :);    
    imshow_intensity_features(I);

    % show backprojected result
    subplot(2, ncols, j+ncols)
    I = selected_backprojected(j, :);
    imshow_intensity_features(I);
end

%% Exercise 1.5: plotting the variance kept in the PCA dimensions
% Create cummulative variance plots for all three feature sets.
% You will need to complete the code in
%    plot_PCA_cumulative_variance.m

pca_int = compute_PCA(all_train_int);
pca_lrf = compute_PCA(all_train_lrf);
pca_hog = compute_PCA(all_train_hog);

figure(1);
clf;
subplot(1,3,1)
plot_PCA_cumulative_variance(pca_int);
title('Intensity');

subplot(1,3,2)
plot_PCA_cumulative_variance(pca_lrf);
title('LRF');

subplot(1,3,3)
plot_PCA_cumulative_variance(pca_hog);
title('HOG');

