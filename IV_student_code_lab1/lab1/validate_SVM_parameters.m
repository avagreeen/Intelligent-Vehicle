% Train a Linear SVM with different parameters C, and for each trained
% model evaluate its performance on both the training and the testing data.
% The function returns a struct with the evaluation results for each
% parameter C.
function results = validate_SVM_parameters(Cs, train_labels, train_features, test_labels, test_features)

results = [];

% try out all of the given values of C
n = numel(Cs);
for p = 1:n
    C = Cs(p);
    fprintf('----------------------------------------------\n');
    fprintf('step %d / %d : C = %.3f\n', p, n, C);
    fprintf('----------------------------------------------\n');
    
    % train model
    svm_model = train_SVM(train_labels, train_features, C);

    % evaluate on train data
    dval_on_train = evaluate_SVM(svm_model, train_features);
    err_train = mean(sign(dval_on_train) ~= train_labels);
    
    % evaluate on test data
    dval_on_test = evaluate_SVM(svm_model, test_features);
    err_test = mean(sign(dval_on_test) ~= test_labels);
    
    res = struct;
    res.C = C;
    res.model = svm_model;
    res.dval_on_train = dval_on_train;
    res.dval_on_train = dval_on_test;
    res.err_train = err_train;
    res.err_test = err_test;
    
    % store result
    results = [results, res];
end

end