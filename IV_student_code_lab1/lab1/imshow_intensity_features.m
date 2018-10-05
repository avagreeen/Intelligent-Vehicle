function imshow_intensity_features(I)
% Visualize a 1250 dimensional feature vector I with gray-scale intensity
% values as an image using imshow.
% The height of this image will be 50 pixels.
% The width of this image will be 25 pixels.

% Hints: you will need to use RESHAPE, and IMSHOW.
% Let IMSHOW automatically rescale the intensity features such that
% the minimum intensity values is black and the maximum value white.
% Use HELP IMSHOW to get information on how to do this.
%
% Do NOT use CLF here since we might be drawing in a subplot here.


% ----------------------
%  YOUR CODE GOES HERE! 
    I=reshape(I,50,25);
    imshow(I,[]);
end
% ----------------------

