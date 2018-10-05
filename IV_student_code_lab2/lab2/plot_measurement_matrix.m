function plot_measurement_matrix(measurements)
    % Visualize all the measurments over time in a single figure
    % This function first concatenate the measurements of all time 
    % steps in a single T x M matrix, such that row t in this matrix
    % contains the measurements of time t
    %
    D = cat(1, measurements.dists);

    figure(2);
    clf
    imagesc(D')
    xlabel('time \rightarrow')
    ylabel('sensor ray')
    title('measured distance by sensor rays over time')   
    colormap default
    set(gca, 'YDir', 'normal')
    
    c = colorbar;
    ylabel(c,'measured distance')
    
    drawnow
end