function in_collision = check_collision(robot, q, link_radius, sphere_centers, sphere_radii, cuboids, resolution)
    % 首先，从 cuboids 矩阵中分离出长方体的起点和长宽高
    cuboid_origin = cuboids(:, 1:3);
    cuboid_ckg = cuboids(:, 4:6);

    x1 = [0 0 0]';%第一个关节

    T2 = robot.A(1,q) ;
    x2 = T2.t;%第二个关节位置

    T3 = robot.A(1:2,q);
    x3 = T3.t;%第三个关节位置

    T4 = robot.A(1:3,q);
    x4 = T4.t;%第4个关节位置
    
    T5 = robot.A(1:4,q);
    x5 = T5.t;%第5个关节位置

    T6 = robot.A(1:5,q);
    x6 = T6.t;%第6个关节位置

    T7 = robot.A(1:6,q);
    x7 = T7.t;%末端位置


    if nargin < 7
        resolution = 11;
    end
    ticks = linspace(0, 1, resolution);
    n = length(ticks);
    x12 = repmat(x1, 1, n) + repmat(x2 - x1, 1, n) .* repmat(ticks, 3, 1); %在1和2两个关节之间取11个点，判断杆是否发生碰撞
    x23 = repmat(x2, 1, n) + repmat(x3 - x2, 1, n) .* repmat(ticks, 3, 1);
    x34 = repmat(x3, 1, n) + repmat(x4 - x3, 1, n) .* repmat(ticks, 3, 1);
    x45 = repmat(x4, 1, n) + repmat(x5 - x4, 1, n) .* repmat(ticks, 3, 1);
    x56 = repmat(x5, 1, n) + repmat(x6 - x5, 1, n) .* repmat(ticks, 3, 1);
    x67 = repmat(x6, 1, n) + repmat(x7 - x6, 1, n) .* repmat(ticks, 3, 1);
    points = [x12 x23 x34 x45 x56 x67];
    
    in_collision = false;
    % 1. 球体碰撞检测 (不变)
    for i = 1:size(sphere_centers, 1)
        if any(sum((points - repmat(sphere_centers(i,:)', 1, size(points, 2))).^2, 1) < (link_radius + sphere_radii(i)).^2)
            in_collision = true;
            break;
        end
    end
    
    % 如果没有和球体碰撞，再检测和长方体的碰撞
    if(~in_collision)
        % 2. 长方体碰撞检测 (已修正)
        for i = 1:size(cuboid_origin, 1)
            % 定义长方体的最小和最大坐标
            cuboid_min_corner = cuboid_origin(i,:);
            cuboid_max_corner = cuboid_origin(i,:) + cuboid_ckg(i,:);
            
            % 将长方体的边界向外扩展 link_radius 的距离，形成一个更大的“安全包围盒”
            expanded_min_corner = cuboid_min_corner - link_radius;
            expanded_max_corner = cuboid_max_corner + link_radius;

            % 检查是否有任何连杆上的采样点进入了这个“扩展后的包围盒”
            if any( points(1,:) > expanded_min_corner(1) & points(1,:) < expanded_max_corner(1) & ...
                    points(2,:) > expanded_min_corner(2) & points(2,:) < expanded_max_corner(2) & ...
                    points(3,:) > expanded_min_corner(3) & points(3,:) < expanded_max_corner(3) )
                in_collision = true;
                break; % 一旦检测到碰撞，立即跳出循环
            end
        end    
    end
end