function in_collision = check_edge(robot, q_start, q_end, link_radius, sphere_centers, sphere_radii, cuboids, resolution)
    if nargin < 8
        resolution = 11;
    end
    ticks = linspace(0, 1, resolution)';
    n = length(ticks);
    configs = repmat(q_start, n, 1) + repmat(q_end - q_start, n, 1) .* repmat(ticks, 1, length(q_start));
    
    in_collision = false;
    for i = 1:n
        if check_collision(robot, configs(i,:), link_radius, sphere_centers, sphere_radii, cuboids)
            in_collision = true;
            break
        end
    end
end