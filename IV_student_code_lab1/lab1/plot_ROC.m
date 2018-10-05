function h = plot_ROC(gt_labels, decision_vals, varargin)

    % compute ROC curve x,y points
    roc_vals = compute_ROC(gt_labels, decision_vals);

    h = plot(roc_vals(:,1), roc_vals(:,2), varargin{:});
    xlabel('False Positive rate');
    ylabel('True Positive rate');
    
    % pass on return value?
    if nargout < 1;
        clear h
    end
end