function state_occupancy = determine_occupied_cells(sp, occupancy_2d)

% define vehicle extent
veh_long = 4.9; % longitudinal size (meter)
veh_lat = 2.7; % lateral size (meter)
veh_size = round([veh_lat / sp.dim_width(1), veh_long / sp.dim_width(2)])-1;
vert_occ = ones(veh_size);
vert_occ = padarray(vert_occ, floor(min(veh_size)+(max(veh_size)-veh_size)/2), 0);

% determine legal states
theta_size = sp.dim_size(3); % number of cells in 3rd dimension
state_occupancy = repmat(occupancy_2d, [1, 1, theta_size]);
vehicle_occ = nan([size(vert_occ) theta_size]);

for j = 1:theta_size
    % rotate by vehicle occupancy map by state angle
    theta = sp.dim_centers{3}(j)/pi*180;
    theta = round(theta*10)/10; % improve precision
    vehicle_occ(:,:,j) = imrotate(double(vert_occ), -theta, 'bilinear', 'crop') > 0;
    
    % remove illegal spatial positions at this angle
    state_occupancy(:,:,j) = conv2(state_occupancy(:,:,j), vehicle_occ(:,:,j), 'same');
    state_occupancy(:,:,j) = state_occupancy(:,:,j) > 0;
end
state_occupancy = repmat(state_occupancy, 1, 1, 1, sp.dim_size(4));
