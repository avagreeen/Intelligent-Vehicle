% Return the HOG features and corresponding image regions for a test frame.
%
% The `sequence` struct contains information for N region proposals,
% but to avoid redundancy computes all HOG cells on the image once.
% This function then combines the HOG cells into feature vectors for the
% (overlapping) region proposals.
%
% Input:
%  sequence        : provided struct with HOG data, and region proposal information
%  frame_id        : target frame for which to obtain all HOG features
%
% Output:
%   frame_test_hog : NxM matrix with per row a HOG feature of a region proposal
%   frame_img_rects: 4xN matrix with the pixel coordinates of the corresponding regions
%                    in the frame. Each column stores [x1;y1;x2;y2].
%                    The region runs from x1 to x2 horizontally,
%                    and from y1 to y2 vertically.
function [frame_test_hog, frame_img_rects] = get_sequence_frame_features(sequence, frame_id)

    % get some sequence properties
    I_offsets = sequence.img_offsets;
    num_feats = prod(sequence.hog_featsize);

    frame_img_rects = cat(2, sequence.img_rects{:});

    num_rects = size(frame_img_rects, 2);
    frame_test_hog = NaN(num_rects, num_feats);

    % get the HOG cells computed over the whole image at different pixel
    % offsets
    image_hogs_by_offset = sequence.frames(frame_id).Hogs;
    image_rects_by_offset = sequence.hog_rects;

    % At each pixel offset, 
    rid = 0;
    for offset_id = 1:size(I_offsets, 2)
        I_Hog = image_hogs_by_offset{offset_id};
        Hrects = image_rects_by_offset{offset_id};
        
        % extract HOG for all regions at this pixel offset
        for j = 1:size(Hrects, 2);
            rid = rid + 1;
            Hx1 = Hrects(1,j); Hy1 = Hrects(2,j);
            Hx2 = Hrects(3,j); Hy2 = Hrects(4,j);

            % extract HOG features for current region proposal
            Ir_Hog = I_Hog(Hy1:Hy2, Hx1:Hx2, :);
            
            % store the result
            frame_test_hog(rid,:) = Ir_Hog(:)';
        end
    end
end
