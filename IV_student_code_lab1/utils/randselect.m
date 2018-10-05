% Given M probabilities (summing to 1), select N indices, with replacement
%
% For example:
%    probs = [.4, .5, .1] 
%    idxs = randselect(probs, 100);
% Then idxs is a 100-dimensional vector with values distributed approximately
%    40% the value 1
%    50% the value 2
%    10% the value 3
function idxs = randselect(probs, N)
    if nargin < 2; N = 1; end
    
    cprobs = cumsum([0; probs(:)]);
    idxs = arrayfun(@(j) find(rand(1) < cprobs, 1)-1, 1:N);
end