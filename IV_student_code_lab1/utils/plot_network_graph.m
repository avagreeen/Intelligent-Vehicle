function plot_network_graph(G, XY, varargin)
% G: n x n connectivity matrix
% XY: n x 2 spatial position (2D) of each node


    [rows, cols] = find(G);
    pairs = [rows, cols];
    mask = rows < cols;
    pairs = pairs(mask, :);

    pairs(:,3) = 1;
    idxs = pairs(:,:)';
    lines_xy = XY(idxs,:);
    lines_xy(3:3:end) = NaN;

    plot(lines_xy(:,1), lines_xy(:,2), varargin{:});
end