function [confMat, classErr] = compute_confusion_matrix(true_labels, est_labels)
    % confusion matrix
    confMat=zeros(2,2);

    assert(all(true_labels == -1 | true_labels == +1))
    assert(all(est_labels == -1 | est_labels == +1))
    
    for i = 1:numel(true_labels)
        % for confusion matrix, we map the label of the non-pedestrian class
        % -1 to matrix index 2

        tl = true_labels(i);    
        if (tl == -1)
            tl = 2;
        end

        cl = est_labels(i);
        if (cl == -1)
            cl = 2;
        end
        
        % add to confusion matrix
        confMat(tl, cl) = confMat(tl, cl)+1;
    end


    % classification error is given by the second diagonal 
    % of the confusion matrix
    classErr = (confMat(1,2) + confMat(2,1)) ./ sum(confMat(:));

    % if no variables should return, just display the result
    if nargout == 0
        fprintf('--- confusion matrix ---\n');
        disp(confMat);
        fprintf('error: %.2f\n', classErr);
        fprintf('------------------------\n');
    end
end