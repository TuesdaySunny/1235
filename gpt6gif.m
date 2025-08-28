%% 主函数：simulate2D - 二维动态RRT避障仿真

function simulate2D()
clc;
clear all;
close all;
    % 初始化二维环境参数
    env.xlim = [0, 100];            % 环境区域X范围
    env.ylim = [0, 100];            % 环境区域Y范围
    startPos = [2, 2];            % 起点坐标
    goalPos  = [90, 90];          % 目标点坐标
    numObstacles = 8;            % 障碍物数量
    obsRadius = 10;             % 障碍物半径 (统一设定)
    maxSteps = 500;              % 仿真最多步数（避免无限循环）
    robotStep = 5;             % 机器人每步移动距离
    
    % 创建初始动态障碍物集合
    obstacles = initObstacles2D(numObstacles, env, obsRadius);
    
    % 图形窗口设置
    figureHandle = figure; % 获取图形句柄，方便后面使用
    clf; hold on; axis equal;
    axis([env.xlim(1) env.xlim(2) env.ylim(1) env.ylim(2)]);
    xlabel('X'); ylabel('Y'); title('二维动态RRT路径规划');
    
    % --- GIF输出初始化 ---
    gifFilename = 'dynamic_rrt_simulation.gif'; % 定义GIF文件名
    frameDelay = 0.1; % 每帧之间的延迟时间（秒），可以根据需要调整，建议与pause()时间匹配或稍快
    firstFrame = true; % 标记是否为第一帧
    % --- GIF输出初始化结束 ---

    % 初始化路径和机器人位置
    pathPoints = [];    % 当前路径点序列（含平滑）
    robotPos = startPos;
    
    % 主循环：逐步移动障碍物和机器人，并必要时重新规划路径
    stepCount = 0;
    reachedGoal = false;
    while ~reachedGoal && stepCount < maxSteps
        fprintf('Step %d, RobotPos: [%.2f %.2f], Remaining Path Points: %d\n', ...
    stepCount, robotPos(1), robotPos(2), size(pathPoints, 1));
        stepCount = stepCount + 1;
        
        % 1. 更新动态障碍物位置
        obstacles = updateObstacles2D(obstacles, env);
        
        % 2. 检查当前路径在新环境下是否仍然安全，如无路径或路径被阻挡则触发重规划
        needReplan = isempty(pathPoints);
        if ~needReplan
            % 若已有路径，检查该路径上是否存在与新障碍物位置的碰撞
            
            for k = 1:size(pathPoints,1)
                if ~collisionFreePoint2D(pathPoints(k,:), obstacles)
                    needReplan = true;
                    break;
                end
            end
        end
        
        % 3. 如需要，则使用改进RRT算法重新规划从当前机器人位置到目标的路径
        if needReplan
            % 清除图形并绘制当前环境（障碍物、起点、终点等）
            cla; 
            drawObstacles2D(obstacles);
            plot(robotPos(1), robotPos(2), 'bo', 'MarkerFaceColor','b', 'MarkerSize',5); % 机器人当前位置
            plot(goalPos(1), goalPos(2), 'rp', 'MarkerFaceColor','r', 'MarkerSize',10);  % 目标点
            drawnow;
            
            % 执行改进RRT路径规划，获取原始路径点序列
            rawPath = improvedRRT2D(robotPos, goalPos, obstacles, env);
           fprintf('rawPath节点数 = %d\n', size(rawPath, 1)); disp('rawPath = ');
disp(rawPath);
            fprintf('RRT路径点数量：%d\n', size(rawPath, 1));
            if isempty(rawPath)
                warning('RRT规划失败，未找到路径！');
                  % --- 捕获失败帧并写入GIF ---
                frame = getframe(figureHandle);
                im = frame2im(frame);
                [imind, cm] = rgb2ind(im, 256);
                if firstFrame
                    imwrite(imind, cm, gifFilename, 'gif', 'Loopcount', inf, 'DelayTime', frameDelay);
                    firstFrame = false;
                else
                    imwrite(imind, cm, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', frameDelay);
                end
                 % --- GIF写入结束 ---
                break;
            end
            
            % 路径剪枝：移除冗余中间节点（直线可达的节点）
            prunedPath = prunePath2D(rawPath, obstacles);
            
            % B样条平滑路径：生成平滑曲线路径点
            pathPoints = smoothPath2D(prunedPath);
            
            % 在图形上绘制RRT生成的原始路径（细线）和平滑后路径（粗线）
            if size(rawPath,1) > 1
                plot(rawPath(:,1), rawPath(:,2), 'g--', 'LineWidth',1);  % 原始路径 (虚线)
            end
            plot(pathPoints(:,1), pathPoints(:,2), 'm-', 'LineWidth',2);   % 平滑路径 (实线)
            % 重新绘制起终点以确保其显示在顶层
            plot(robotPos(1), robotPos(2), 'bo', 'MarkerFaceColor','b', 'MarkerSize',5);
            plot(goalPos(1), goalPos(2), 'rp', 'MarkerFaceColor','r', 'MarkerSize',10);
            drawnow;
            % 注意：重规划后不立即移动机器人，直接进入下一次循环判断

            % --- 捕获重规划后的帧并写入GIF ---
            frame = getframe(figureHandle);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256);
            if firstFrame
                imwrite(imind, cm, gifFilename, 'gif', 'Loopcount', inf, 'DelayTime', frameDelay);
                firstFrame = false;
            else
                imwrite(imind, cm, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', frameDelay);
            end
            % --- GIF写入结束 ---
            pause(frameDelay); % 等待一下，与GIF帧延迟一致
            % 重规划步中不移动机器人，继续下一循环
            continue;
        end
        
        % 4. 按既有路径移动机器人一小步
        % 将机器人朝着路径下一个目标点移动 robotStep 距离
        % 如果剩余距离不足一步，则直接到达下一个路径点
% 4. 按既有路径移动机器人一小步
% 4. 按既有路径移动机器人一小步
if size(pathPoints,1) >= 2
    nextPoint = pathPoints(2,:);
    
    % 实时检测机器人当前位置到下一个路径点之间是否发生碰撞
    if ~collisionFreeSegment2D(robotPos, nextPoint, obstacles)
        disp('检测到运动路径段发生碰撞，准备重新规划！');
        pathPoints = [];    % 清空路径触发重新规划
         % --- 捕获碰撞检测失败帧并写入GIF ---
                 frame = getframe(figureHandle);
                 im = frame2im(frame);
                 [imind, cm] = rgb2ind(im, 256);
                 if firstFrame
                     imwrite(imind, cm, gifFilename, 'gif', 'Loopcount', inf, 'DelayTime', frameDelay);
                     firstFrame = false;
                 else
                     imwrite(imind, cm, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', frameDelay);
                 end
                 % --- GIF写入结束 ---
                 pause(frameDelay);
        continue;           % 跳过移动，进入下一轮重新规划
    end

    % 否则，按路径正常移动
    dirVec = nextPoint - robotPos;
    dist = norm(dirVec);
    if dist <= robotStep
        robotPos = nextPoint;
        pathPoints(1:2,:) = [];
    else
        moveVec = robotStep * (dirVec / dist);
        robotPos = robotPos + moveVec;
    end
end

        % 5. 更新可视化：绘制当前环境、路径和机器人位置
        cla;
        drawObstacles2D(obstacles);
        if ~isempty(pathPoints)
            plot(pathPoints(:,1), pathPoints(:,2), 'm-', 'LineWidth',2);    % 剩余平滑路径
        end
        plot(robotPos(1), robotPos(2), 'bo', 'MarkerFaceColor','b', 'MarkerSize',5);
        plot(goalPos(1), goalPos(2), 'rp', 'MarkerFaceColor','r', 'MarkerSize',10);
        drawnow;
% --- 捕获当前帧并写入GIF ---
        frame = getframe(figureHandle); % 从指定句柄的figure捕获
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256); % 转换为索引图像和颜色图

        if firstFrame
            imwrite(imind, cm, gifFilename, 'gif', 'Loopcount', inf, 'DelayTime', frameDelay);
            firstFrame = false; % 第一次写入后，将标志设为false
        else
            imwrite(imind, cm, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', frameDelay); % 追加模式写入后续帧
        end
        % --- GIF写入结束 ---

        pause(0.1);   % ← 加在这里

        % 6. 检查是否到达目标
        if norm(robotPos - goalPos) < 1e-1
            reachedGoal = true;
            disp('机器人已到达目标点！');
            % 可以选择在这里再捕获并写入最后一帧GIF
             frame = getframe(figureHandle);
             im = frame2im(frame);
             [imind, cm] = rgb2ind(im, 256);
             imwrite(imind, cm, gifFilename, 'gif', 'WriteMode', 'append', 'DelayTime', frameDelay);
        end
    end
end

%% 函数：improvedRRT2D - 改进RRT算法规划二维路径
% 基于当前机器人位置start和目标位置goal，规划一条避开障碍物obstacles的路径。
% 使用自适应步长扩展和动态目标采样提高效率。返回路径上的节点坐标序列（包含起点和终点）。
function path = improvedRRT2D(start, goal, obstacles, env)
    % 参数设置
    maxIter = 2000;         % 最大迭代次数（规划失败则返回空路径）
    stepSizeInit = 10;     % 初始步长
    stepSizeMin  = 5;     % 最小步长
    stepSizeMax  = 20;     % 最大步长
    goalSampleRate = 0.1;   % 动态目标采样概率（10%概率直接采样目标）
    goalThreshold = 10;    % 判断到达目标的距离阈值
    
    % RRT树的节点结构：每个元素含字段 pos=[x,y], parent=index
    node(1).pos = start;
    node(1).parent = 0;
    nodeCount = 1;
    
    % 当前步长变量（根据情况自适应调整）
    stepSize = stepSizeInit;
    
    % 图形绘制：初始化树节点和边的显示
    % 假设当前figure处于 hold on 状态以绘制树生长过程
    plot(start(1), start(2), 'bo', 'MarkerFaceColor','b', 'MarkerSize',5);  % 起点标记
    plot(goal(1), goal(2), 'rp', 'MarkerFaceColor','r', 'MarkerSize',10);   % 目标标记
    
    % 迭代扩展RRT树
    goalIndex = -1;  % 用于记录目标节点索引
    for iter = 1:maxIter
        % 随机采样新点（有一定概率在目标附近采样，以加快收敛）
        if rand < goalSampleRate
            samplePos = goal;  % 直接选择目标点作为采样
        else
            % 在环境范围内均匀随机采样一点
            samplePos = [env.xlim(1) + rand*(env.xlim(2)-env.xlim(1)), ...
                         env.ylim(1) + rand*(env.ylim(2)-env.ylim(1))];
        end
        
        % 找到树中最近的已有节点
        nearestIndex = 1;
        minDist = norm(samplePos - node(1).pos);
        for j = 2:nodeCount
            d = norm(samplePos - node(j).pos);
            if d < minDist
                minDist = d;
                nearestIndex = j;
            end
        end
        nearestPos = node(nearestIndex).pos;
        
        % 计算从最近节点指向采样点的方向向量
        direction = samplePos - nearestPos;
        distToSample = norm(direction);
        if distToSample < eps
            continue;  % 避免采样点与最近点重合
        end
        direction = direction / distToSample;  % 单位方向
        
        % 沿该方向尝试扩展节点（自适应步长）
        % 分步检测碰撞：以小步长段推进，确保路径上的每个小段均无碰撞
        maxExtendDist = min(stepSize, distToSample);  % 本次最大扩展距离（不超过当前步长或直接到采样点）
        extendStep = maxExtendDist / 10;  % 将扩展路径分成10份进行碰撞检测
        newPos = nearestPos; 
        movedDist = 0;
        collisionFlag = false;
        for m = 1:10
            if movedDist + extendStep >= maxExtendDist
                % 最后一段直接到达目标延伸距离
                nextDist = maxExtendDist - movedDist;
            else
                nextDist = extendStep;
            end
            candidatePos = newPos + nextDist * direction;
            % 边界检查（确保候选点在环境范围内）
            candidatePos(1) = min(max(candidatePos(1), env.xlim(1)), env.xlim(2));
            candidatePos(2) = min(max(candidatePos(2), env.ylim(1)), env.ylim(2));
            % 碰撞检测：候选点是否与任何障碍物碰撞
            if ~collisionFreePoint2D(candidatePos, obstacles)
                collisionFlag = true;
                break;
            end
            % 无碰撞，则接受该小步推进
            newPos = candidatePos;
            movedDist = movedDist + nextDist;
            % 如果已经推进到最大扩展距离，则退出小步循环
            if movedDist >= maxExtendDist - 1e-6
                break;
            end
        end
        
        % 根据碰撞检测结果确定新节点位置
        if collisionFlag
            % 碰撞发生，若尚未移动任何距离，则放弃本次采样扩展
            if movedDist < 1e-6
                % 调整步长（环境复杂，减少步长以尝试更细粒度搜索）
                stepSize = max(stepSize * 0.7, stepSizeMin);
                continue;
            end
            % 若碰撞在中途发生，则newPos为最后一个无碰撞的位置
            %（可以将此位置添加为新节点）
        else
            % 无碰撞，全步长成功
            % 根据环境开阔程度增加步长（加快扩展速度），上限为stepSizeMax
            stepSize = min(stepSize * 1.1, stepSizeMax);
        end
        
        % 添加新节点到RRT树
        nodeCount = nodeCount + 1;
        node(nodeCount).pos = newPos;
        node(nodeCount).parent = nearestIndex;
        % 绘制树的扩展边（从父节点到新节点的线段）
        parentPos = node(nearestIndex).pos;
        plot([parentPos(1), newPos(1)], [parentPos(2), newPos(2)], 'b-');
        plot(newPos(1), newPos(2), 'bo', 'MarkerSize',3);  % 绘制新节点
        drawnow limitrate;  % 实时更新绘图（限制刷新率以提高性能）
        
        % 检查是否到达目标区域
        if norm(newPos - goal) < goalThreshold
            % 将目标点作为最终节点加入树（如果尚未非常接近也可直接将 newPos 视为到达）
         nodeCount = nodeCount + 1;
goalIndex = nodeCount;
node(goalIndex).pos = goal;
node(goalIndex).parent = nodeCount - 1;  % ✅ 正确地连接到 newPos 节点
            % 绘制最后一段从newPos到目标的线
            plot([newPos(1), goal(1)], [newPos(2), goal(2)], 'b-');
            plot(goal(1), goal(2), 'rp', 'MarkerFaceColor','r', 'MarkerSize',10);
            drawnow;
            break;
        end
    end
    
    % 若在规定迭代内未找到路径，则返回空
    if goalIndex == -1
        path = [];
        return;
    end
    
    % 回溯路径：从目标节点沿parent指针返回起点
    path = [];
    idx = goalIndex;
    if idx > length(node)
        % 上面为了绘制方便，可能多加了一次goalIndex，此处调整
        idx = length(node);
    end
    while idx ~= 0
        path = [node(idx).pos; path];
        idx = node(idx).parent;
    end
end

%% 函数：prunePath2D - 路径剪枝去冗余
% 输入为包含起点到终点的路径点序列，尝试移除中间多余节点。
% 如果路径上相隔较远的两点之间可直接连线无碰撞，则去除它们之间的节点。
function newPath = prunePath2D(path, obstacles)
    if isempty(path)
        newPath = [];
        return;
    end
    newPath = [path(1,:)];  % 保留起点
    i = 1;
    % 尝试直接跳跃连接路径后面的点
    while i < size(path,1)
        j = size(path,1);
        % 从路径末端往前找，找到最远的、可以与当前newPath末端连通的点
        for k = j:-1:(i+1)
            if collisionFreeSegment2D(path(i,:), path(k,:), obstacles)
                j = k;
                break;
            end
        end
        % 将能够直连的最远点加入新路径
        newPath = [newPath; path(j,:)]; %#ok<AGROW>
        i = j;
        if i == size(path,1)
            break;  % 已到终点
        end
    end
end

%% 函数：smoothPath2D - B样条平滑路径
% 使用三次样条（B样条）对路径进行平滑，返回密集采样的平滑轨迹点。
% 注意：需要Curve Fitting Toolbox提供的cscvn函数。
function smoothPts = smoothPath2D(path)
    if size(path,1) < 3
        % 若路径点太少，不需平滑或无法平滑，直接返回
        smoothPts = path;
        return;
    end
    % 构造2xN矩阵的点序列供 cscvn 使用
    points = path';
    splineCurve = cscvn(points);
    % 等距参数取样，生成平滑曲线上的一系列点
    numSamples = max(100, 10 * size(path,1));  % 采样点数量（根据路径长度适当选择）
    t = linspace(splineCurve.breaks(1), splineCurve.breaks(end), numSamples);
    vals = fnval(splineCurve, t);
    smoothPts = vals';  % 转置为 [N x 2] 每行一个点
end

%% 函数：collisionFreePoint2D - 检查点是否与任一障碍物碰撞
% 如果point与所有圆形障碍物都保持至少其半径距离，则视为无碰撞。
function free = collisionFreePoint2D(point, obstacles)
    free = true;
    for obs = obstacles
        dist = norm(point - obs.pos);
        if dist < obs.radius - 1e-6   % 距离小于半径则视为碰撞（带一点容差）
            free = false;
            return;
        end
    end
end

%% 函数：collisionFreeSegment2D - 检查线段是否与障碍物碰撞
% 通过离散采样线段上的点来近似检测线段与圆形障碍的碰撞。
function free = collisionFreeSegment2D(p1, p2, obstacles)
    free = true;
    segLength = norm(p2 - p1);
    if segLength < 1e-6
        free = true;
        return;
    end
    % 以路径段长度的1%为步长进行采样检查（至少采样100点）
    step = max(0.01 * segLength, 0.1);
    t = 0:step:segLength;
    if t(end) < segLength
        t = [t, segLength];
    end
    for dist = t
        point = p1 + (dist/segLength) * (p2 - p1);
        if ~collisionFreePoint2D(point, obstacles)
            free = false;
            return;
        end
    end
end

%% 函数：initObstacles2D - 初始化随机移动障碍物
% 创建指定数量的障碍物，随机分布在环境中，每个障碍物有随机移动速度。
function obstacles = initObstacles2D(n, env, radius)
    obstacles = struct('pos', {}, 'vel', {}, 'radius', {});
    for i = 1:n
        % 随机初始位置 (避开边界以防一开始就越界)
        x = env.xlim(1) + radius + rand*(env.xlim(2) - env.xlim(1) - 2*radius);
        y = env.ylim(1) + radius + rand*(env.ylim(2) - env.ylim(1) - 2*radius);
        % 随机速度向量 (幅度在[-0.3, 0.3]范围内)
        % 随机速度向量 (幅度在[-0.3, 0.3]范围内)  <-- 这是旧的注释，实际范围是[-0.4, 0.4]
        vx = (rand*0.8 - 0.4);
        vy = (rand*0.8 - 0.4);
        obstacles(i).pos = [x, y];
        obstacles(i).vel = [vx, vy];
        obstacles(i).radius = radius;
    end
end

%% 函数：updateObstacles2D - 更新障碍物位置（实现简单的边界反弹运动）
function obstacles = updateObstacles2D(obstacles, env)
    for i = 1:length(obstacles)
        % 更新位置
        obstacles(i).pos = obstacles(i).pos + obstacles(i).vel;
        % 碰到边界则反弹，并限制在边界内
        if obstacles(i).pos(1) < env.xlim(1) + obstacles(i).radius
            obstacles(i).pos(1) = env.xlim(1) + obstacles(i).radius;
            obstacles(i).vel(1) = -obstacles(i).vel(1);
        elseif obstacles(i).pos(1) > env.xlim(2) - obstacles(i).radius
            obstacles(i).pos(1) = env.xlim(2) - obstacles(i).radius;
            obstacles(i).vel(1) = -obstacles(i).vel(1);
        end
        if obstacles(i).pos(2) < env.ylim(1) + obstacles(i).radius
            obstacles(i).pos(2) = env.ylim(1) + obstacles(i).radius;
            obstacles(i).vel(2) = -obstacles(i).vel(2);
        elseif obstacles(i).pos(2) > env.ylim(2) - obstacles(i).radius
            obstacles(i).pos(2) = env.ylim(2) - obstacles(i).radius;
            obstacles(i).vel(2) = -obstacles(i).vel(2);
        end
    end
end

%% 函数：drawObstacles2D - 绘制二维圆形障碍物
function drawObstacles2D(obstacles)
    theta = 0:0.1:2*pi+0.1;
    for obs = obstacles
        cx = obs.pos(1); cy = obs.pos(2); r = obs.radius;
        X = cx + r*cos(theta);
        Y = cy + r*sin(theta);
        fill(X, Y, [0.8 0.4 0.4], 'EdgeColor', 'r');  % 绘制填充圆形表示障碍物
    end
end
