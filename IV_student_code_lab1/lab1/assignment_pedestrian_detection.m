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

%% load training data
% add / set paths
data_path = fullfile(IV_BASE_PATH, 'lab1_data/');
img_dir = fullfile(data_path, 'test_sequence/');

% load data
disp('Loading data ...');
load(fullfile(data_path, 'new_data_labels.mat'));
load(fullfile(data_path, 'new_data_hog.mat'));

% load training and testing data
train_labels = [ped_train_labels; garb_train_labels];
all_train_hog = [ped_train_hog; garb_train_hog];

test_labels = [ped_test_labels; garb_test_labels];
all_test_hog = [ped_test_hog; garb_test_hog];

%% Exercise 3.1 & 3.2: Train and evaluate a SVM and HOG
% Use the provided training data in all_train_hog to train a classifier.
% You can use all_test_hog to evaluate classifiers.

% ----------------------
%  YOUR CODE GOES HERE! 
% ----------------------


%% load video sequence information
% The test sequence consists of a directory with images, and a `sequence`
% struct which contains pre-computed HOG cells on the whole image.
% In particular, the HOG cells are on 12x12 pixel regions, and bin into 
% 9 distinct orientations.
% Since we need to test many overlapping region proposals, it is
% inefficient to store the HOG feature vectors for each region
% independently, especially since maby overlapping regions share the exact
% same cells. Therefore, we provide a function get_sequence_frame_features
% below that combines the precomputed HOG cells in the struct for each 
% region proposal.
% In other words, you do not need to use the `sequence` struct directly,
% but can just use the output from get_sequence_frame_features below.

% this is the directory where the image frames are stored
setup_filename = sprintf('setup_c12_o9.mat');

% load test sequence HOG feature information
load(fullfile(img_dir, setup_filename));

%% loop through all frames in the test video

T = sequence.T; % sequence length (e.g. T = 41 frames)

for frame_id = 1:T
    fprintf('frame %2d / %2d ...\n', frame_id, T);

    % load frame image
    img_name = sequence.frames(frame_id).img_name;
    img_path = fullfile(img_dir, img_name);
    
    I = imread(img_path);
    
    % get the HOG features and corresponding region proposals for this frame.
    % If there are N region proposals, then
    %   frame_test_hog : NxM matrix with per row a HOG feature of a region proposal
    %   frame_img_rects: 4xN matrix with the pixel coordinates of the corresponding regions
    %                    in the frame. Each column stores [x1;y1;x2;y2].
    %                    The region runs from x1 to x2 horizontally,
    %                    and from y1 to y2 vertically.
    [frame_test_hog, frame_img_rects] = get_sequence_frame_features(sequence, frame_id);
    
    %% Exercise 3.3: Apply your classifier on the region proposals
    % Use your trained classifier on the HOG features of all the region proposals here.
    
    % a vector containing the decision value of your classifier for
    % all the target region proposals.
    decision_val = rand(size(frame_test_hog,1),1); % dummy values
    
    % a decision threshold to compare the decision_val values to
    %   this value depends on the classifier that you use, but can
    %   also be tuned to alter the sensitivity towards
    %   false positives or false negatives.
    decision_thresh = 0.99; % dummy threshold, let's 1% of dummy decision values through
    
    % ----------------------
    %  YOUR CODE GOES HERE! 
    % ----------------------

    
    %% visualize
    
    % visualize the result
    sfigure(1);
    cla; % NOTE: using cla instead of clf for smoother animation
    imshow(I);
    
    % draw all region proposals for which the decision values exceeds
    % the decision threshold.
    hold all;
    rids = find(decision_val > decision_thresh)';
    for rid = rids
        Ix1 = frame_img_rects(1,rid); Iy1 = frame_img_rects(2,rid);
        Ix2 = frame_img_rects(3,rid); Iy2 = frame_img_rects(4,rid);

        plot([Ix1 Ix2 Ix2 Ix1 Ix1], [Iy1 Iy1 Iy2 Iy2 Iy1], 'g-', 'Tag', 'rect');
    end
    
    % force matlab to update the figure before continuing with the outer loop
    drawnow
end
