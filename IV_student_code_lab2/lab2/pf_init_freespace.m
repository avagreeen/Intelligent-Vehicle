% Generate N 
function particles = pf_init_freespace(N, map)
    particles = map.freespace(:,randi(size(map.freespace, 2), 1, N));
    particles(1:2,:) = particles(1:2,:) + (rand(2,N)-.5);
    particles(3,:) = rand(1, N) * 2*pi; %0 + mod(randi(4, 1, N)/4 * 2*pi, 2*pi); % 4 directions
end