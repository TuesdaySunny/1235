%% 3D Dynamic Obstacle Avoidance RRT Simulation - Main Script

% 清空环境并初始化随机数种子（可选）
clear; close all; clc;
rng('shuffle');  % 随机种子

%% 参数设置
% 环境边界 [xmin xmax; ymin ymax; zmin zmax]
bounds = [0 20; 0 20; 0 10];

% 起点和终点 (在边界范围内)
start = [1, 1, 1];
goal  = [18, 18, 9];

% RRT算法参数
maxIter = 500;           % RRT最大迭代次数
stepSize = 1.0;          % 初始步长
minStep = 0.5;           % 最小步长
maxStep = 3.0;           % 最大步长
goalSampleRate = 0.1;    % 目标采样概率（10%概率直接采样目标）
threshDist = 1.0;        % 判定到达目标的距离阈值

% 动态障碍物设置 (示例: 两个球体和一个长方体)
obstacles = [];
% 障碍物1: 球体
obstacles(1).type = 'sphere';
obstacles(1).radius = 1.5;
obstacles(1).center = [5, 5, 2];            % 初始中心
obstacles(1).velocity = [0.5, 0.3, 0.2];    % 速度向量
obstacles(1).color = [1, 0, 0];            % 红色 (RGB)
% 障碍物2: 球体
obstacles(2).type = 'sphere';
obstacles(2).radius = 2.0;
obstacles(2).center = [15, 5, 3];
obstacles(2).velocity = [-0.4, 0.6, 0.0];
obstacles(2).color = [1, 0, 0];            % 红色
% 障碍物3: 长方体 (轴对齐的矩形盒)
obstacles(3).type = 'cuboid';
obstacles(3).size = [3, 2, 4];             % 尺寸 [长, 宽, 高]
obstacles(3).center = [10, 15, 5];
obstacles(3).velocity = [0.2, -0.5, 0.3];
obstacles(3).color = [0, 0.7, 0];          % 绿色 (稍微调整避免纯绿太亮)

% 预计算长方体的半长、半宽、半高，便于碰撞检测和可视化
for obs = 1:length(obstacles)
    if strcmp(obstacles(obs).type, 'cuboid')
        obstacles(obs).halfSize = obstacles(obs).size / 2;
    end
end

%% 仿真初始化
% 创建图形窗口和轴
figure('Name','3D RRT Dynamic Obstacle Avoidance');
axis equal;
axis([bounds(1,:) bounds(2,:) bounds(3,:)]);
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); grid on;
set(gca, 'Projection', 'orthographic');
hold on;

% 初始化变量
robotPos = start;        % 机器人当前位置
pathPoints = [];         % 当前跟随的路径点序列 (平滑后的)
pathIndex = 1;           % 机器人当前路径点索引
lastTreeNodes = [];      % 上一次规划的树节点集
lastTreeParents = [];    % 上一次规划的树父节点集

%% 主循环：迭代更新动态障碍物和机器人运动
maxSteps = 500;          % 最大仿真步数，防止无限循环
for step = 1:maxSteps
    % 1. 更新动态障碍物位置
    obstacles = updateObstacles(obstacles, bounds);  % 更新障碍物中心并处理反弹

    % 2. 检查是否需要重新规划路径:
    %    情况1: 当前无路径（初始时或先前规划失败）
    %    情况2: 当前路径还有剩余，但下一步会与障碍物碰撞
    needReplan = false;
    if isempty(pathPoints)
        needReplan = true;
    else
        % 如果存在路径，检查机器人当前位置到下一个路径点的直线路径是否安全
        if pathIndex < size(pathPoints,1)
            nextPoint = pathPoints(pathIndex+1, :);
            % 检测从robotPos到nextPoint是否碰撞任何障碍物
            if checkCollision(robotPos, nextPoint, obstacles)
                needReplan = true;
            end
        else
            % 已到达路径终点（应是目标）
            needReplan = false;
        end
    end

    % 3. 如需要则调用RRT重新规划新路径
    if needReplan
        [nodes, parents, rawPath] = RRTplan(robotPos, goal, obstacles, bounds, stepSize, minStep, maxStep, goalSampleRate, threshDist, maxIter);
        if isempty(rawPath)
            % 没有找到路径：机器人停留在原地，尝试下一次迭代
            fprintf('第%d步：未找到路径，机器人等待...\n', step);
        else
            % 路径后处理：剪枝和平滑
            prunedPath = prunePath(rawPath, obstacles);
            pathPoints = smoothPath(prunedPath);
            pathPoints(end, :) = goal;  % 确保路径终点为目标点

            pathIndex = 1;  % 新路径，从头开始跟随
            % 保存树用于可视化
            lastTreeNodes = nodes;
            lastTreeParents = parents;
            fprintf('第%d步：重新规划路径，路径长度（折线段）=%d\n', step, size(prunedPath,1));
        end
    end

    % 4. 沿当前路径移动机器人
if ~isempty(pathPoints)
    if pathIndex < size(pathPoints,1)
        pathIndex = pathIndex + 1;
        robotPos = pathPoints(pathIndex, :);
    elseif pathIndex == size(pathPoints,1)
        robotPos = pathPoints(end, :);  % ← 最后一个点也走到
    end
end
 cla; hold on;
    % 绘制动态障碍物
                [xSphere, ySphere, zSphere] = sphere(30);  % ✅ 更高分辨率（默认是20，更圆）

    for obs = 1:length(obstacles)
        if strcmp(obstacles(obs).type, 'sphere')
            % 绘制球体障碍物
sphCenter = obstacles(obs).center;
R = obstacles(obs).radius;
mesh(R*xSphere + sphCenter(1), R*ySphere + sphCenter(2), R*zSphere + sphCenter(3), ...
     'FaceColor', obstacles(obs).color, 'EdgeColor', 'none', 'FaceAlpha', 0.4);

        elseif strcmp(obstacles(obs).type, 'cuboid')
            % 绘制长方体障碍物
            c = obstacles(obs).center;
            h = obstacles(obs).halfSize;
            % 计算长方体6个面并用patch绘制
            % 面1: 底面 (z = c(3)-h(3))
            x = [c(1)-h(1), c(1)+h(1), c(1)+h(1), c(1)-h(1)];
            y = [c(2)-h(2), c(2)-h(2), c(2)+h(2), c(2)+h(2)];
            z = [c(3)-h(3), c(3)-h(3), c(3)-h(3), c(3)-h(3)];
            patch(x, y, z, obstacles(obs).color, 'FaceAlpha', 0.3);
            % 面2: 顶面 (z = c(3)+h(3))
            z = [c(3)+h(3), c(3)+h(3), c(3)+h(3), c(3)+h(3)];
            patch(x, y, z, obstacles(obs).color, 'FaceAlpha', 0.3);
            % 面3: 前面 (y = c(2)-h(2))
            x = [c(1)-h(1), c(1)+h(1), c(1)+h(1), c(1)-h(1)];
            y = [c(2)-h(2), c(2)-h(2), c(2)-h(2), c(2)-h(2)];
            z = [c(3)-h(3), c(3)-h(3), c(3)+h(3), c(3)+h(3)];
            patch(x, y, z, obstacles(obs).color, 'FaceAlpha', 0.3);
            % 面4: 后面 (y = c(2)+h(2))
            y = [c(2)+h(2), c(2)+h(2), c(2)+h(2), c(2)+h(2)];
            patch(x, y, z, obstacles(obs).color, 'FaceAlpha', 0.3);
            % 面5: 左面 (x = c(1)-h(1))
            x = [c(1)-h(1), c(1)-h(1), c(1)-h(1), c(1)-h(1)];
            y = [c(2)-h(2), c(2)+h(2), c(2)+h(2), c(2)-h(2)];
            z = [c(3)-h(3), c(3)-h(3), c(3)+h(3), c(3)+h(3)];
            patch(x, y, z, obstacles(obs).color, 'FaceAlpha', 0.3);
            % 面6: 右面 (x = c(1)+h(1))
            x = [c(1)+h(1), c(1)+h(1), c(1)+h(1), c(1)+h(1)];
            patch(x, y, z, obstacles(obs).color, 'FaceAlpha', 0.3);
        end
    end

    % 绘制RRT树结构（如果有可用）
    if ~isempty(lastTreeNodes)
        % 遍历每个节点，画出其与父节点的连线
        for i = 2:size(lastTreeNodes,1)  % 从2开始，因为1是根节点无父
            pi = lastTreeParents(i);
            if pi ~= 0
                X = [ lastTreeNodes(pi,1), lastTreeNodes(i,1) ];
                Y = [ lastTreeNodes(pi,2), lastTreeNodes(i,2) ];
                Z = [ lastTreeNodes(pi,3), lastTreeNodes(i,3) ];
                plot3(X, Y, Z, '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 1); % 灰色线
            end
        end
    end

    % 绘制当前规划路径
    if ~isempty(pathPoints)
        plot3(pathPoints(:,1), pathPoints(:,2), pathPoints(:,3), 'b-', 'LineWidth', 2);
    end

% 如果已到达目标，把当前位置画成红色，与终点重合，不额外再画“机器人点”
plot3(start(1), start(2), start(3), 'go', 'MarkerFaceColor','g', 'MarkerSize', 8);
plot3(robotPos(1), robotPos(2), robotPos(3), 'mo', 'MarkerFaceColor','m', 'MarkerSize', 8);
plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerFaceColor','r', 'MarkerSize', 10);

drawnow;
    pause(0.05);  % 暂停以便观察动画，数值可调节仿真速度

    % 如果已达到目标，则退出循环
  if norm(robotPos - goal) < 0.2

        fprintf('机器人在第%d步到达目标！\n', step);
        robotPos = goal;  % 确保精确在目标
        % 确保路径列表最后一个点是目标
        pathIndex = size(pathPoints,1);
        pathPoints(end,:) = goal;
        reached = true;
        % 可在此设置，例如暂停或结束
        % break;  % 这里选择跳出循环
        break;
    else
        reached = false;
    end

    % 5. 可视化当前状态
   
end

% 仿真结束后，如需要，可将最终路径输出或用于机械臂轨迹跟踪
if reached
    disp('规划完成，机器人已到达目标点！');
else
    disp('仿真结束，机器人未到达目标点。');
end
fprintf('当前位置: %.6f %.6f %.6f\n', robotPos(1), robotPos(2), robotPos(3));
fprintf('目标位置: %.6f %.6f %.6f\n', goal(1), goal(2), goal(3));



function newPath = prunePath(path, obstacles)
% prunePath 对路径进行剪枝，移除不必要的中间节点
% path: M x 3 的坐标序列（包含起点和终点）
% obstacles: 障碍物集合，用于碰撞检测
%
% 输出:
%   newPath: 剪枝后的路径坐标序列

    if isempty(path)
        newPath = path;
        return;
    end
    % 初始化新路径列表
    newPath = path(1, :);  % 从起点开始
    currentIndex = 1;
    totalPoints = size(path, 1);
    % 从起点开始逐步寻找可以直接跳到的最远节点
    while currentIndex < totalPoints
        if currentIndex == totalPoints
            % 已到终点
            break;
        end
        % 在剩余路径中从最后一个点往回找，可以直接到达的最远点
        nextIndex = currentIndex + 1;
        for j = totalPoints:-1:(currentIndex+1)
            if ~checkCollision(path(currentIndex, :), path(j, :), obstacles)
                nextIndex = j;
                break;
            end
        end
        % 将该可达点加入新路径
        newPath(end+1, :) = path(nextIndex, :);
        % 更新当前索引为新加入的点索引
        currentIndex = nextIndex;
        % 如果已经到达终点，则结束
        if currentIndex == totalPoints
            break;
        end
    end
end
function collision = checkCollision(q1, q2, obstacles)
% checkCollision 检查从点q1到q2的线段是否与任何障碍物相撞
% q1, q2: 1x3 起点和终点坐标
% obstacles: 障碍物结构数组

    collision = false;
    % 线段向量参数表示: P(t) = q1 + t*(q2 - q1), t∈[0,1]
    v = q2 - q1;
    v_len_sq = sum(v.^2);
    % 遍历每个障碍物
    for obs = 1:length(obstacles)
        type = obstacles(obs).type;
        if strcmp(type, 'sphere')
            % 球体障碍物
            center = obstacles(obs).center;
            R = obstacles(obs).radius;
            % 若q1本身在球内
            if norm(q1 - center) <= R
                collision = true; return;
            end
            % 计算线段到球心的最近距离
            if v_len_sq < 1e-9
                % q1和q2几乎重合
                dist = norm(q1 - center);
            else
                t_star = dot(center - q1, v) / v_len_sq;
                if t_star < 0
                    closest = q1;
                elseif t_star > 1
                    closest = q2;
                else
                    closest = q1 + t_star * v;
                end
                dist = norm(closest - center);
            end
            if dist <= R
                collision = true; return;
            end

        elseif strcmp(type, 'cuboid')
            % 长方体障碍物 (轴对齐)
            c = obstacles(obs).center;
            h = obstacles(obs).halfSize;
            % 定义盒子的最小和最大范围
            minB = c - h;
            maxB = c + h;
            % 若q1在盒子内
            if all(q1 >= minB & q1 <= maxB)
                collision = true; return;
            end
            % 使用参数法计算相交
            t0 = 0; t1 = 1;
            % 检查x轴方向
            if abs(v(1)) < 1e-9
                if q1(1) < minB(1) || q1(1) > maxB(1)
                    continue;  % 平行于x轴平面且起点在范围外，无碰撞
                end
            else
                t_enter = (minB(1) - q1(1)) / v(1);
                t_exit  = (maxB(1) - q1(1)) / v(1);
                if t_enter > t_exit
                    temp = t_enter; t_enter = t_exit; t_exit = temp;
                end
                if t_enter > t0, t0 = t_enter; end
                if t_exit < t1, t1 = t_exit; end
                if t0 > t1, continue; end  % 没有交集，检查下一个障碍
            end
            % 检查y轴方向
            if abs(v(2)) < 1e-9
                if q1(2) < minB(2) || q1(2) > maxB(2)
                    continue;
                end
            else
                t_enter = (minB(2) - q1(2)) / v(2);
                t_exit  = (maxB(2) - q1(2)) / v(2);
                if t_enter > t_exit
                    temp = t_enter; t_enter = t_exit; t_exit = temp;
                end
                if t_enter > t0, t0 = t_enter; end
                if t_exit < t1, t1 = t_exit; end
                if t0 > t1, continue; end
            end
            % 检查z轴方向
            if abs(v(3)) < 1e-9
                if q1(3) < minB(3) || q1(3) > maxB(3)
                    continue;
                end
            else
                t_enter = (minB(3) - q1(3)) / v(3);
                t_exit  = (maxB(3) - q1(3)) / v(3);
                if t_enter > t_exit
                    temp = t_enter; t_enter = t_exit; t_exit = temp;
                end
                if t_enter > t0, t0 = t_enter; end
                if t_exit < t1, t1 = t_exit; end
                if t0 > t1, continue; end
            end
            % 若经过以上筛选，线段与长方体相交
            collision = true; return;
        end
    end
end
function [nodes, parents, path] = RRTplan(start, goal, obstacles, bounds, stepSize, minStep, maxStep, goalSampleRate, threshDist, maxIter)
% RRTplan 在三维空间规划路径（避开障碍），返回树节点、父节点索引和从start到goal的路径点列表（若找到）。
%
% 输入:
%   start, goal: 1x3 起点和终点坐标
%   obstacles: 障碍物结构体数组（包含类型、位置和大小）
%   bounds: 3x2 环境边界 [xmin xmax; ymin ymax; zmin zmax]
%   stepSize: 初始步长
%   minStep, maxStep: 最小和最大步长限制
%   goalSampleRate: 每次迭代直接采样目标的概率 (0~1)
%   threshDist: 判定到达目标的距离阈值
%   maxIter: 最大迭代次数
%
% 输出:
%   nodes: N x 3 的节点坐标列表（包含起点和可能的终点）
%   parents: N x 1 的父节点索引数组（parents(i)是第i个节点的父节点索引，起点父索引为0）
%   path: K x 3 的坐标列表，从起点到终点的路径（若未找到路径则为空）

    % 初始化节点列表和父索引
    nodes = zeros(maxIter+1, 3);   % 预分配(最多maxIter+1个节点，包括起点)
    parents = zeros(maxIter+1, 1);
    nodes(1, :) = start;
    parents(1) = 0;
    nodeCount = 1;
    found = false;
    goalIndex = -1;

    % RRT主循环
    for iter = 1:maxIter
        % 动态目标采样：按照一定概率选择目标点作为随机点
        if rand() < goalSampleRate
            q_rand = goal;
        else
            % 在边界内随机采样一点 [x, y, z]
            q_rand = [ ...
                bounds(1,1) + (bounds(1,2)-bounds(1,1)) * rand(), ...
                bounds(2,1) + (bounds(2,2)-bounds(2,1)) * rand(), ...
                bounds(3,1) + (bounds(3,2)-bounds(3,1)) * rand() ];
        end

        % 1. 在已有节点集合中找到距离 q_rand 最近的节点 q_near
        nearestIndex = 1;
        minDist = inf;
        for j = 1:nodeCount
            d = norm(q_rand - nodes(j,:));
            if d < minDist
                minDist = d;
                nearestIndex = j;
            end
        end
        q_near = nodes(nearestIndex, :);

        % 2. 从 q_near 朝向 q_rand 延伸一步得到 q_new
        direction = q_rand - q_near;
        dist = norm(direction);
        if dist == 0
            continue;  % q_rand恰好与已有节点重合，跳过
        end
        direction = direction / dist;  % 单位方向向量

        % 确定实际步长：如果目标距离小于当前步长，则直接到目标点，否则按照步长延伸
        step = stepSize;
        if dist < step
            newStep = dist;
        else
            newStep = step;
        end
        q_new = q_near + newStep * direction;

        % 边界检查：如果q_new超出环境边界，则调整到边界上
        q_new(1) = max(bounds(1,1), min(bounds(1,2), q_new(1)));
        q_new(2) = max(bounds(2,1), min(bounds(2,2), q_new(2)));
        q_new(3) = max(bounds(3,1), min(bounds(3,2), q_new(3)));

        % 如果新的q_new与q_near太接近（可能由于随机点在边界导致），则跳过
        if norm(q_new - q_near) < 1e-6
            continue;
        end

        % 3. 碰撞检测：检查从 q_near 到 q_new 的直线是否与任何障碍物碰撞
        if checkCollision(q_near, q_new, obstacles)
            % 发生碰撞，调整步长（减小）并放弃此方向扩展
            stepSize = max(minStep, stepSize * 0.7);
            continue;
        end

        % 无碰撞，可以添加新节点到树
        nodeCount = nodeCount + 1;
        nodes(nodeCount, :) = q_new;
        parents(nodeCount) = nearestIndex;

        % 碰撞反馈：成功扩展，适当增大步长
        stepSize = min(maxStep, stepSize * 1.1);

        % 4. 检查是否达到目标
        if norm(q_new - goal) < threshDist
            % 检查从q_new直接连接到goal是否碰撞
            if ~checkCollision(q_new, goal, obstacles)
                % 可以直接连接到目标
                nodeCount = nodeCount + 1;
                nodes(nodeCount, :) = goal;
                parents(nodeCount) = nodeCount - 1;  % 父节点是刚加入的q_new
                goalIndex = nodeCount;
                found = true;
                break;
            end
        end
    end

    % 如果循环结束但未添加goal节点且q_new达到目标条件未满足，则检查是否直接采样到目标
    if ~found
        % 尝试：如果最后添加的节点与目标足够接近且连线无碰撞，也可认为成功
        % （此步通常不需要，因为上面已处理，但作为双重保证）
        for j = 1:nodeCount
            if norm(nodes(j,:) - goal) < threshDist && ~checkCollision(nodes(j,:), goal, obstacles)
                nodeCount = nodeCount + 1;
                nodes(nodeCount, :) = goal;
                parents(nodeCount) = j;
                goalIndex = nodeCount;
                found = true;
                break;
            end
        end
    end

    % 如果找到路径，则回溯获取路径点列表
    if found
        % 剪裁nodes和parents数组大小到实际用量
        nodes = nodes(1:nodeCount, :);
        parents = parents(1:nodeCount);
        % 从goalIndex反向回溯到起点
        pathIdxList = goalIndex;
        while parents(pathIdxList(1)) ~= 0
            pathIdxList = [parents(pathIdxList(1)); pathIdxList]; %#ok<AGROW> 
        end
        % 将索引转换为坐标路径
        path = nodes(pathIdxList, :);
    else
        % 未找到路径
        path = [];
        nodes = nodes(1:nodeCount, :);
        parents = parents(1:nodeCount);
    end
end
function smoothPts = smoothPath(path)
% smoothPath 使用三次B样条曲线对路径进行平滑，输出平滑后的离散点
% path: N x 3 剪枝后的路径点
% smoothPts: L x 3 平滑路径上的采样点序列

    if size(path, 1) < 3
        % 如果路径点太少（0、1或2点），无法拟合复杂曲线，直接返回原路径
        smoothPts = path;
        return;
    end
    % 准备点序列供 cscvn 使用，需要转置为 3xN 矩阵
    pts = path';
    % 构建三次自然样条曲线
    curve = cscvn(pts);
    % 根据曲线片段数选择采样点数，每段采样10个点
    numSegments = length(curve.breaks) - 1;
    sampleCount = numSegments * 10;
    tt = linspace(curve.breaks(1), curve.breaks(end), sampleCount);
    out = fnval(curve, tt);
    smoothPts = out.';
end
function obstacles = updateObstacles(obstacles, bounds)
% updateObstacles 更新动态障碍物的位置，并在碰到边界时反弹
% obstacles: 障碍物结构体数组（包含center和velocity）
% bounds: 3x2 环境边界

    for obs = 1:length(obstacles)
        % 更新位置: 新中心 = 旧中心 + 速度
        newCenter = obstacles(obs).center + obstacles(obs).velocity;
        % 获取边界最小和最大值
        x_min = bounds(1,1); x_max = bounds(1,2);
        y_min = bounds(2,1); y_max = bounds(2,2);
        z_min = bounds(3,1); z_max = bounds(3,2);
        % 根据障碍物类型，确定尺寸（球的半径或盒子半径）
        if strcmp(obstacles(obs).type, 'sphere')
            extents = [obstacles(obs).radius, obstacles(obs).radius, obstacles(obs).radius];
        elseif strcmp(obstacles(obs).type, 'cuboid')
            extents = obstacles(obs).halfSize;
        end
        % 检查并处理边界反弹 (每个轴分别判断)
        % X方向
        if newCenter(1) - extents(1) < x_min
            newCenter(1) = x_min + extents(1);
            obstacles(obs).velocity(1) = -obstacles(obs).velocity(1);
        elseif newCenter(1) + extents(1) > x_max
            newCenter(1) = x_max - extents(1);
            obstacles(obs).velocity(1) = -obstacles(obs).velocity(1);
        end
        % Y方向
        if newCenter(2) - extents(2) < y_min
            newCenter(2) = y_min + extents(2);
            obstacles(obs).velocity(2) = -obstacles(obs).velocity(2);
        elseif newCenter(2) + extents(2) > y_max
            newCenter(2) = y_max - extents(2);
            obstacles(obs).velocity(2) = -obstacles(obs).velocity(2);
        end
        % Z方向
        if newCenter(3) - extents(3) < z_min
            newCenter(3) = z_min + extents(3);
            obstacles(obs).velocity(3) = -obstacles(obs).velocity(3);
        elseif newCenter(3) + extents(3) > z_max
            newCenter(3) = z_max - extents(3);
            obstacles(obs).velocity(3) = -obstacles(obs).velocity(3);
        end
        % 更新中心位置
        obstacles(obs).center = newCenter;
    end
end
