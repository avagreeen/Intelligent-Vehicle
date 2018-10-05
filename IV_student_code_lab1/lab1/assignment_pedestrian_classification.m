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
%
% NOTE: if you get an *ERROR* that IV_BASE_PATH, or some custom function, is
% not defined, then you probably forget to run the startup.m script once,
% as described in the README that came with the code !!!
% This script should add the utility directories to your path, and setup
% the IV_BASE_PATH variable.

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

test_labels = [ped_test_labels; garb_test_labels];
all_test_hog = [ped_test_hog; garb_test_hog];
all_test_lrf = [ped_test_lrf; garb_test_lrf];
all_test_int = [ped_test_int; garb_test_int];

% shuffle test data to avoid accidental biases
pidxs = randperm(size(all_test_hog,1));
all_test_hog = all_test_hog(pidxs,:);
all_test_lrf = all_test_lrf(pidxs,:);
all_test_int = all_test_int(pidxs,:);
test_labels = test_labels(pidxs);

%% Construct PCA space
numPCAComponents = 20;

% esimtate PCA
pca_hog = compute_PCA(all_train_hog, numPCAComponents);
pca_lrf = compute_PCA(all_train_lrf, numPCAComponents);
pca_int = compute_PCA(all_train_int, numPCAComponents);

all_train_hog_pca  = apply_PCA(pca_hog, all_train_hog);
all_train_lrf_pca  = apply_PCA(pca_lrf, all_train_lrf);
all_train_int_pca  = apply_PCA(pca_int, all_train_int);

all_test_hog_pca  = apply_PCA(pca_hog, all_test_hog); % --- HOG ---
all_test_lrf_pca  = apply_PCA(pca_lrf, all_test_lrf); % --- LRF ---
all_test_int_pca  = apply_PCA(pca_int, all_test_int);  % --- Intensity ---


%% Exercise 2.1: Train and evaluate linear SVMs
% You will need to complete the code in
%    train_SVM.m
%    evaluate_SVM.m

disp('Training linear SVMs ...');

svm_hog = train_SVM(train_labels, all_train_hog);
svm_lrf = train_SVM(train_labels, all_train_lrf);
svm_int = train_SVM(train_labels, all_train_int);

% Evaluate linear SVMs
disp('Evaluating linear SVMs ...');

% compute estimated labels on test data
[dval_hog_svm] = evaluate_SVM(svm_hog, all_test_hog); % HOG features
[dval_lrf_svm] = evaluate_SVM(svm_lrf, all_test_lrf); % LRF features
[dval_int_svm] = evaluate_SVM(svm_int, all_test_int); % Intensity features

% compute evaluation statistics
disp('HOG features')
compute_confusion_matrix(test_labels, sign(dval_hog_svm));

disp('LRF features')
compute_confusion_matrix(test_labels, sign(dval_lrf_svm));

disp('Intensity features')
compute_confusion_matrix(test_labels, sign(dval_int_svm));
 
%% Exercise 2.2: Train and evaluate GMM
% You will need to complete the code in
%    train_GMM.m
%    evaluate_GMM.m

num_components = 5;

gmm_hog = train_GMM(train_labels, all_train_hog_pca, num_components);
gmm_lrf = train_GMM(train_labels, all_train_lrf_pca, num_components);
gmm_int = train_GMM(train_labels, all_train_int_pca, num_components);

disp('Evaluating GMM/Bayes ...');
 
[dval_hog_gmm] = evaluate_GMM(gmm_hog, all_test_hog_pca);
[dval_lrf_gmm] = evaluate_GMM(gmm_lrf, all_test_lrf_pca);
[dval_int_gmm] = evaluate_GMM(gmm_int, all_test_int_pca);

% compute evaluation statistics
disp('HOG features')
compute_confusion_matrix(test_labels, (dval_hog_gmm > .5)*2-1);

disp('LRF features')
compute_confusion_matrix(test_labels, (dval_lrf_gmm > .5)*2-1);

disp('Intensity features')
compute_confusion_matrix(test_labels, (dval_int_gmm > .5)*2-1);

%% Exercise 2.3: evaluate using ROC curves
% The ROC curves are computed within plot_ROC by the function compute_ROC
% You will need to complete the code in
%    compute_ROC.m

disp('Computing ROC curves ...');

% plot ROC curves
disp('ROC plotting ...');
figure(1);
clf;
hold all
plot_ROC(test_labels, dval_hog_svm, 'r', 'DisplayName', 'HOG/linSVM');
plot_ROC(test_labels, dval_lrf_svm, 'b', 'DisplayName', 'LRF/linSVM');
plot_ROC(test_labels, dval_int_svm, 'k', 'DisplayName', 'Int/linSVM');
plot_ROC(test_labels, dval_hog_gmm, 'r--', 'DisplayName', 'HOG/GMM');
plot_ROC(test_labels, dval_lrf_gmm, 'b--', 'DisplayName', 'LRF/GMM');
plot_ROC(test_labels, dval_int_gmm, 'k--', 'DisplayName', 'Int/GMM');
title('ROC Curves for Feature/Classifier Combinations');
xlabel('False Positive Rate');
ylabel('True Positive Rate');
legend_by_displayname;
grid on;
disp('done');

%% Exercise 2.4 & 2.5: Overfitting
% for each C, let's train the SVM on the training data, and
% then evaluate on the following two dataset:
%  1. the SAME training data
%  2. the test data
% and compare the reported performance.

% try out different values of C in different orders of magnitude,
%   range from .001 up to 1000
Cs = 10 .^ [-3:.5:3];

% for each parameter C, train a classifier on the test set,
%    and EVALUATE it on both TRAIN and TEST data.
results = validate_SVM_parameters(Cs, train_labels, all_train_hog, test_labels, all_test_hog);

errs_train = [results.err_train]; % classification error on training data
errs_test = [results.err_test]; % classification error on test data

% report best (= lowest) errors for both train and test
[~, best_train_idx] = min(errs_train);
[~, best_test_idx] = min(errs_test);
fprintf('best result on training data : C = %.2e\t err = %.3f\n', Cs(best_train_idx), errs_train(best_train_idx));
fprintf('best result on test data     : C = %.2e\t err = %.3f\n', Cs(best_test_idx), errs_test(best_test_idx));

% plot error as a function of C
figure(2);
clf;
hold all
h = plot(Cs, errs_train, '.-', 'DisplayName', 'error on train data');
plot(Cs(best_train_idx), errs_train(best_train_idx), '.', 'MarkerSize', 30, 'Color', h.Color); % show best result
h = plot(Cs, errs_test, '.-', 'DisplayName', 'error on test data');
plot(Cs(best_test_idx), errs_test(best_test_idx), '.', 'MarkerSize', 30, 'Color', h.Color); % show best result
legend_by_displayname
set(gca, 'XScale', 'log')
grid on
xlabel('regularization C')
ylabel('error rate')
ylim([0 1])

%% Exercise 2.6: Fusion 
% You will need to complete the code in
%    fuse_decision_values.m

% compute two fused decision value vectors,
%   dval_fuse_mean: for mean fusion
%   dval_fuse_max: for max fusion
[dval_fuse_mean, dval_fuse_max] = fuse_decision_values(dval_hog_svm, dval_lrf_svm);

% plot ROC curves
disp('ROC plotting ...');
figure(3);
clf;
hold all
plot_ROC(test_labels, dval_hog_svm, 'r', 'DisplayName', 'HOG/linSVM');
plot_ROC(test_labels, dval_lrf_svm, 'b', 'DisplayName', 'LRF/linSVM');
plot_ROC(test_labels, dval_fuse_mean, 'g--', 'DisplayName', 'mean fusion');
plot_ROC(test_labels, dval_fuse_max, 'g:', 'DisplayName', 'max fusion');
grid on
legend_by_displayname('Location', 'SouthEast')
xlim([0 1])
title('Multi-feature fusion');
