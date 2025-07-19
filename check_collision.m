function in_collision = check_collision(robot, q, link_radius, sphere_centers, sphere_radii, cuboids, resolution)
    % ���ȣ��� cuboids �����з��������������ͳ����
    cuboid_origin = cuboids(:, 1:3);
    cuboid_ckg = cuboids(:, 4:6);

    x1 = [0 0 0]';%��һ���ؽ�

    T2 = robot.A(1,q) ;
    x2 = T2.t;%�ڶ����ؽ�λ��

    T3 = robot.A(1:2,q);
    x3 = T3.t;%�������ؽ�λ��

    T4 = robot.A(1:3,q);
    x4 = T4.t;%��4���ؽ�λ��
    
    T5 = robot.A(1:4,q);
    x5 = T5.t;%��5���ؽ�λ��

    T6 = robot.A(1:5,q);
    x6 = T6.t;%��6���ؽ�λ��

    T7 = robot.A(1:6,q);
    x7 = T7.t;%ĩ��λ��


    if nargin < 7
        resolution = 11;
    end
    ticks = linspace(0, 1, resolution);
    n = length(ticks);
    x12 = repmat(x1, 1, n) + repmat(x2 - x1, 1, n) .* repmat(ticks, 3, 1); %��1��2�����ؽ�֮��ȡ11���㣬�жϸ��Ƿ�����ײ
    x23 = repmat(x2, 1, n) + repmat(x3 - x2, 1, n) .* repmat(ticks, 3, 1);
    x34 = repmat(x3, 1, n) + repmat(x4 - x3, 1, n) .* repmat(ticks, 3, 1);
    x45 = repmat(x4, 1, n) + repmat(x5 - x4, 1, n) .* repmat(ticks, 3, 1);
    x56 = repmat(x5, 1, n) + repmat(x6 - x5, 1, n) .* repmat(ticks, 3, 1);
    x67 = repmat(x6, 1, n) + repmat(x7 - x6, 1, n) .* repmat(ticks, 3, 1);
    points = [x12 x23 x34 x45 x56 x67];
    
    in_collision = false;
    % 1. ������ײ��� (����)
    for i = 1:size(sphere_centers, 1)
        if any(sum((points - repmat(sphere_centers(i,:)', 1, size(points, 2))).^2, 1) < (link_radius + sphere_radii(i)).^2)
            in_collision = true;
            break;
        end
    end
    
    % ���û�к�������ײ���ټ��ͳ��������ײ
    if(~in_collision)
        % 2. ��������ײ��� (������)
        for i = 1:size(cuboid_origin, 1)
            % ���峤�������С���������
            cuboid_min_corner = cuboid_origin(i,:);
            cuboid_max_corner = cuboid_origin(i,:) + cuboid_ckg(i,:);
            
            % ��������ı߽�������չ link_radius �ľ��룬�γ�һ������ġ���ȫ��Χ�С�
            expanded_min_corner = cuboid_min_corner - link_radius;
            expanded_max_corner = cuboid_max_corner + link_radius;

            % ����Ƿ����κ������ϵĲ�����������������չ��İ�Χ�С�
            if any( points(1,:) > expanded_min_corner(1) & points(1,:) < expanded_max_corner(1) & ...
                    points(2,:) > expanded_min_corner(2) & points(2,:) < expanded_max_corner(2) & ...
                    points(3,:) > expanded_min_corner(3) & points(3,:) < expanded_max_corner(3) )
                in_collision = true;
                break; % һ����⵽��ײ����������ѭ��
            end
        end    
    end
end