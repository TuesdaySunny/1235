function [path, path_found] = RRT8(robot, q_min, q_max, q_start, q_goal, link_radius, ...
    sphere_center, sphere_radius, cuboids)
    % 定义配置空间的范围
    q_limits = [q_min; q_max];
    
    % 初始化RRT树
    % 每个节点包含当前配置(q)、父节点配置(q_pre)、步长距离(dist)、父节点索引(idx_pre)
    t.v(1).q = q_start;      % 根节点为起点配置
    t.v(1).q_pre = q_start;  % 起点的父节点也是起点
    t.v(1).dist = 0;         % 起点距离为0
    t.v(1).idx_pre = 0;      % 起点没有父节点
    
    % 参数设置
    max_iterations = 50; % RRT搜索的最大迭代次数
 step_size = 1.5;           % 基础步长，用于扩展节点
    Smax = step_size * 2;    % 最大步长
    Smin = 1;              % 最小步长
    K = 0.8;                 % 动态步长缩减系数
    goal_region = 500;       % 判断到达目标的距离阈值
    goalSampleRadius = 1;    % 在目标邻域采样的半径
    goalProximityThreshold = 1; % 目标接近阈值，用于判断采样是否直接取目标
    pmin = 0.2;              % 最小目标偏置采样概率
    pmax = 0.9;              % 最大目标偏置采样概率
    alpha = 0.3;             % 自适应采样参数
    Tfailed = 0;             % 初始化碰撞检测失败次数为0
    Pa = pmin;               % 初始目标采样概率为最小值
    threshold = 0.5; % 当节点到目标的距离小于该阈值时，认为达到了目标

    
    % 初始化路径找到标志和树的节点计数
    path_found = false;      % 初始未找到路径
    count = 1;               % 树的节点计数器，初始为1（起点）

    % 主循环，执行RRT搜索
    for iteration = 1:max_iterations
        % 自适应目标偏置采样
        if rand < Pa
            % 以一定概率从全局空间随机采样
            q_rand = q_limits(1, :) + (q_limits(2, :) - q_limits(1, :)) .* rand(1, 6);
        else
            % 其余情况直接采样目标点
            q_rand = q_goal;
        end

        
        % 找到树中距离随机采样点最近的节点
        dist = arrayfun(@(x) vecnorm(x.q - q_rand, 2, 2), t.v); % 计算每个节点到采样点的距离
        [min_dist, idx] = min(dist);  % 获取最小距离和对应的节点索引
        q_near = t.v(idx).q;          % 最近的节点配置
        
        % 动态步长调整
        delta_q = q_rand - q_near;             % 计算方向向量
        step = min(Smax, step_size * (min_dist / step_size)); % 根据距离调整步长
        q_new = q_near + step * delta_q / norm(delta_q);      % 新节点配置
        
        % 沿着最近节点向随机配置扩展
        delta_q_step = step_size * (q_rand - q_near) / min_dist; % 计算增量
        q_new = q_near + delta_q_step; % 新配置
        robot.plot(q_new); % 绘制扩展后的配置（可视化）
    

        % 检查从q_near到q_new的路径是否无碰撞
        if ~check_edge(robot, q_near, q_new, link_radius, sphere_center, sphere_radius, cuboids)
            % 如果无碰撞，将新节点加入树
            count = count + 1;       % 树的节点计数加1
            t.v(count).q = q_new;    % 存储新节点配置
            t.v(count).q_pre = q_near; % 存储父节点配置
            t.v(count).dist = step; % 记录步长
            t.v(count).idx_pre = idx; % 记录父节点索引
            
            % 可视化扩展的路径
            T = zeros(4, 4, 2);
            T(:, :, 2) = robot.fkine(q_new); % 计算新节点的正向运动学结果
            T(:, :, 1) = robot.fkine(q_near); % 计算父节点的正向运动学结果
            T(:, :, 3) = robot.fkine(q_goal); % 目标配置的正向运动学结果

            plot3(squeeze(T(1, 4, :)), squeeze(T(2, 4, :)), squeeze(T(3, 4, :)), 'k-', 'LineWidth', 1);
            hold on;
            fprintf('树的大小为: %d\n', count); % 输出当前树的节点数量

            % 检查是否到达目标区域
            if norm(q_new - q_goal) < threshold
                path_found = true;  % 标记路径找到
                break;              % 退出主循环
            end


            % 成功扩展后，重置采样概率和碰撞失败计数
            Pa = pmin;
            Tfailed = 0;
        else
            % 扩展失败，增加碰撞失败计数，并调整采样概率
            Tfailed = Tfailed + 1;
            Pa = pmin + (pmax - pmin) * (1 - exp(-alpha * Tfailed));
        end
    end
    
    % 如果找到路径，沿树回溯构造路径
    if path_found
        path.pos(1).q = q_goal; % 将目标点加入路径
        path.pos(2).q = t.v(end).q; % 将最后一个节点加入路径
        pathIndex = t.v(end).idx_pre; % 从最后一个节点的父节点开始回溯
        j = 0; % 路径节点计数
        while pathIndex > 0 % 当父节点索引为正整数时继续回溯
            path.pos(j + 3).q = t.v(pathIndex).q; % 将当前节点加入路径
            pathIndex = t.v(pathIndex).idx_pre; % 更新索引为父节点
            fprintf('回溯到节点: %d\n', pathIndex); % 输出回溯到的节点索引
            j = j + 1;
        end
        path.pos(end + 1).q = q_start; % 将起点加入路径
    else
        % 如果未找到路径，返回0
        path = 0;
    end
end