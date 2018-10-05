% Define the road/lane following scenario
% Input:
%  - rradius : road curvature radius (meter), rradius --> inf = straight
%
% Output:
%  - road : struct containing the x,y points of the different lines
function road = setup_trajectory_scenario(rradius)
if nargin < 1
    rradius = 50; % curvature radius (meter), rradius --> inf = straight
end
rlength = 40; % length of road segment (meter)
rwidth = 4; % width of road / lane (meter)

rsteps = 100; % discretization steps of road representation
rlong = linspace(0, rlength, rsteps); % distance in meters along road

% compute world-coordinates of road left, center and right border
[rx, ry, rtheta] = make_road_xy(rradius, rlong, [0, -rwidth/2, +rwidth/2, -rwidth*1.5]);

% create struct
road = struct;
road.rradius = rradius;
road.rwidth = rwidth;
road.rlength = rlength;
road.rsteps = rsteps;
road.rlong = rlong;

road.rx = rx;
road.ry = ry;
road.rtheta = rtheta;

end