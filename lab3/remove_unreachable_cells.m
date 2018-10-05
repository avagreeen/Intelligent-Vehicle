% Exercise 2.3: Remove occupied vertices from all the lists of reachable vertices.
%
% This function removes edges that connect 'non-occupied' vertices
%   to 'occupied' vertices, given the original list of reachable vertices
%   and information on which vertices are 'occupied'.
%
% Input:
%  - reachable: a 1 x V cell array
%               reachable{v} contains the list of vertex indices of the
%               neighboring vertices reachable from vertex index v.
%               For example, reachable{1} = [4, 7] means that there is
%               are two edges from vertex 1, namely to vertex 4 and to 7.
%  - state_occupancy: a (4D) array containing values 0 and 1 indicating.
%               For example, if state_occupancy(3,6,4,1) == 0, then
%               cell with 4D index [3,6,4,1] is not occupied.
%               NOTE: you can also use a vertex index directly on
%               state_occpancy, i.e. state_occupancy(3235) give the
%               state occupancy of vertex 3235.
%
% Output:
%  - reachable: a 1 x V cell array
%               similar to the input, but now vertices that are
%               occupied have been removed from each list of reachable
%               vertices.
function reachable = remove_unreachable_cells(reachable, state_occupancy)

V = numel(reachable);

% Loop over all vertices v, and remove from the list reachable{v}
%   all neighboring vertex indices for which state_occupancy is true.
% ----------------------
%  YOUR CODE GOES HERE! 
% ----------------------
