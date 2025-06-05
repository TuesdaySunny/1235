% 清除命令窗口中的信息，清除工作区中的所有变量，并关闭所有打开的图形窗口
clc;
clear all;
close all;

%% 定义障碍物

%% 较为复杂障碍物
circleCenter = [50,50,30;120,120,60;60,120,120;110,30,30;120,50,120];
r = [15,20,15,20,15];
cuboids = [ 40,80,30,20,30,60 ;100,120,100,30,30,30  ;90,50,50,50,30,20;20,30,90,40,30,20];


% 
% % 简单障碍物
% circleCenter = [140,100,100];
% r = [20];
% cuboids = [ 60, 0, 0, 40, 200, 80;60,0,100,40,200,100];

% 
% % 复杂障碍物
% circleCenter = [80,80,80;120,110,100;20,50,60;80,140,100;40,130,50;
%      50,60,40;80,50,130;120,50,100;50,50,100;
%      120,70,50;120,120,45;170,60,100;160,100,120;20,20,130;150,80,60;];
% r = [15;20;15;15;15;10;20;15;10;10;10;15;20;15;20];
% cuboids = [ 50, 10, 50, 30, 20, 20;  70, 80, 10, 40, 30, 40 ;40,120,80,20,30,20;150,20,120,30,20,30;130,10,40,30,30,40;20,80,10,20,20,50;40,100,120,40,40,20];

%% 绘制三维障碍物
figure(1);
[xSphere, ySphere, zSphere] = sphere;
for i = 1:length(circleCenter(:, 1))
    mesh(r(i)*xSphere + circleCenter(i, 1), r(i)*ySphere + circleCenter(i, 2), r(i)*zSphere + circleCenter(i, 3));
    hold on;
end
for i = 1:size(cuboids, 1)
    drawCuboid(cuboids(i, :), 'k'); % 绘制长方体
end

% view(3); grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal;

%% 参数初始化
ABC=0;
source = [10, 10, 10]; % 起点坐标
goal = [150, 150, 150]; % 终点坐标
stepsize = 15;
Smax = [];      % 最大步长（初始化为空）
n = 2;
Smin = 4;
K = 0.9;
threshold = 15;
maxIterations = 1000;
maxFailedAttempts = 10000; % 设置最大失败
notFoundCount = 0;  % 未找到目标点的计数器

display = true;            % 设置
searchSize = [200 200 200];% 设置搜索空间
goalSampleRadius = 25;     % 定义在靠近目标时的采样半径
goalProximityThreshold = 45;

% 自适应采样概率相关的参数设置
pmin = 0.1; % 偏向目标的最小采样概率
pmax = 1;
alpha = 0.3; % 控制自适应采样概率变化速率的参数
Tfailed = 0; % 初始化碰撞检测失败次数为0
Pa = pmin;% 初始化采样概率为最小值

% 在图形窗口中绘制起点和终点，分别用绿色和蓝色表示
hold on;

scatter3(source(1), source(2), source(3), 100,"filled", "g"); % 起点
scatter3(goal(1), goal(2), goal(3), 100,"filled", "r"); % 终点

% 初始化RRT树
tic;
RRTree = double([source -1]);
% 初始化失败尝试次数和路径找到的标志
failedAttempts = 0;  % 记录当前失败尝试次数
pathFound = false;   % 路径是否找到的标志
iterationCount = 0;  % 迭代次数计数器

%% 主循环
while iterationCount <= maxIterations  % 当迭代次数没有超过最大迭代次数时循环
    iterationCount = iterationCount + 1;  % 每次循环时增加迭代次数

% 决定是否进行随机采样或目标偏执采样
    if rand < Pa  % Pa 是随机采样的概率
        % 进行随机采样，生成一个随机点，范围在搜索空间内
        sample = rand(1,3) .* searchSize;
    else
        % 目标偏执采样：直接将目标点作为采样点
        sample = goal;
    end

    % 找到RRT树中距离采样点最近的节点
    [A, I] = min(distanceCost(RRTree(:,1:3), sample), [], 1);  % A为最小距离，I为索引
    closestNode = RRTree(I(1), 1:3);  % 取出树中最近的节点的坐标
    % 计算从最近节点到采样点的方向向量，并归一化
    movingVec = [sample(1) - closestNode(1), sample(2) - closestNode(2), sample(3) - closestNode(3)];
    movingVec = movingVec / sqrt(sum(movingVec.^2));  % 归一化向量，使其长度为1

    % 动态调整步长，初始步长为Smax，即基础步长乘以n
    Smax = stepsize * n;  % 最大步长
    currentStepSize = Smax;  % 当前步长初始化为最大值
    newPoint = closestNode + currentStepSize * movingVec;  % 根据步长计算新点位置

    % 尝试使用动态步长从当前节点扩展到采样点

    while currentStepSize >= Smin
        % 如果从当前节点到新点的路径没有碰撞，则接受这个新点
        if checkPath3(closestNode, newPoint, circleCenter, r, cuboids)
            Pa = pmin;
            ABC= Tfailed+ABC;
            Tfailed = 0;
            break;
        else
            currentStepSize = currentStepSize * K;
            newPoint = closestNode + currentStepSize * movingVec;
        end
    end

    % 如果步长小于最小步长，则认为扩展失败
    if currentStepSize < Smin
        Tfailed = Tfailed + 1;
        Pa = pmin + (pmax - pmin) * (1 - exp(-alpha * Tfailed));
        continue;
    end

    % 如果新点距离目标点小于预设的阈值，则认为找到了路径
    if distanceCost(newPoint, goal) < threshold
        pathFound = true;
        break;
    end


    % 将新点加入RRT树，记录其父节点为最近的节点
    RRTree = [RRTree; newPoint I(1)];  % 将新点及其父节点索引添加到树中

    % 如果开启显示模式，则绘制新点和连接线
    if display
        plot3([closestNode(1); newPoint(1)], [closestNode(2); newPoint(2)], [closestNode(3); newPoint(3)], 'LineWidth', 2);  % 绘制连接线
        scatter3(newPoint(1), newPoint(2), newPoint(3), 20, 'k', 'filled');  % 绘制新点
    end
 pause(0.01);  % 暂停0.01秒，便于观察图形更新
end

 runTime=toc;


% 如果找到路径并且显示模式开启，绘制最终连接终点的线段
if display && pathFound
   
    plot3([closestNode(1); goal(1)], [closestNode(2); goal(2)], [closestNode(3); goal(3)]);  % 绘制最终线段
end

%% 构建路径

% 从RRT树中检索路径，根据父节点索引从终点往回追溯到起点
path = goal;  % 初始化路径，首先包含终点
prev = I(1);  % 获取终点节点的父节点索引
while prev > 0  % 当父节点索引有效时（不等于-1，表示还没到起点）
    path = [RRTree(prev, 1:3); path];  % 将父节点的坐标加入路径开头，路径从终点向起点反向构建
    prev = RRTree(prev, 4);  % 更新当前节点为其父节点，继续向上追溯
end

% path = [source; path];  % 在路径的开头添加起点


% 计算从起点到终点的路径长度
pathLength = 0;  % 初始化路径长度为0
for i = 1:length(path(:,1))-1
    % 依次计算路径中每两点之间的距离并累加到总长度
    pathLength = pathLength + distanceCost(path(i, 1:3), path(i+1, 1:3));
end

    AiterationCount=iterationCount;
    BTfailed=ABC;
    Cnode=length(RRTree)+1;
    Dnode=length(path);
    Elength=pathLength;


% 输出RRT搜索的结果，包括迭代次数、路径节点数、路径总长度、RRT树中的总节点数
% fprintf(['Iteration Count总迭代次数=%d \n总碰撞检测失败次数=%d \n' ...
%     'Total nodes初始路径总节点数=%d \nTree nodes初始路径节点数=%d \n' ...
%     'Path Length初始路径总长度=%.2f \n'], ...
%     iterationCount, ABC ,length(RRTree)+1,length(path), pathLength);

% 创建一个新的图形窗口，用于显示RRT树、路径以及障碍物
figure(2);
for i = 1:length(circleCenter(:, 1))
    mesh(r(i)*xSphere + circleCenter(i, 1), r(i)*ySphere + circleCenter(i, 2), r(i)*zSphere + circleCenter(i, 3));
    hold on;
end
for i = 1:size(cuboids, 1)
    drawCuboid(cuboids(i, :), 'k'); % 绘制长方体
end

% view(3); grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal;
hold on;
% 绘制起点和终点
scatter3(source(1), source(2), source(3), 100,"filled", "g");  % 绘制起点，绿色
scatter3(goal(1), goal(2), goal(3), 100,"filled", "r");        % 绘制终点，蓝色

% 显示RRT树中的所有节点和父节点之间的连接
for i = 2:size(RRTree, 1)
    parentIndex = RRTree(i, 4);  % 获取当前节点的父节点索引
    % 绘制当前节点与其父节点之间的连接线
    plot3([RRTree(i,1); RRTree(parentIndex, 1)], [RRTree(i,2); RRTree(parentIndex, 2)], [RRTree(i,3); RRTree(parentIndex, 3)], 'LineWidth', 2, 'color', [1, 0.5, 0]);  % 使用橙色线条
end

% 绘制树中所有节点的位置，节点用黑色小圆点表示
scatter3(RRTree(:,1), RRTree(:,2), RRTree(:,3), 10, 'k', 'filled');

% 显示从RRT树中提取的初始路径，用淡紫色线条表示
plot3(path(:,1), path(:,2), path(:,3), 'LineWidth', 2, 'color', [0.6, 0.4, 0.8]);  % 绘制路径
scatter3(path(:,1), path(:,2), path(:,3), 15, 'k', 'filled');  % 绘制路径中的节点

if ~pathFound
    notFoundCount = notFoundCount + 1;  % 增加未找到目标点的计数
end

hold off;

function h=distanceCost(a,b)         % 定义一个名为distanceCost的函数，接收两个参数a和b，分别代表两个点或两组点的坐标
	h = sqrt(sum((a-b).^2, 2));       % 计算a和b两点之间的欧氏距离。首先计算a和b对应元素的差，然后将差值平方，
    % 对所有维度求和（sum的第二个参数2表示沿数组的每一行求和），最后对求和结果开平方根得到距离h
end                                 % 结束函数定义
function feasible = checkPath3(p1, p2, circleCenter, r, cuboids)
    feasible = true;
    direction = (p2 - p1) / norm(p2 - p1);
    for t = 0:0.5:norm(p2 - p1)
        point = p1 + t * direction;
        % 检查球形障碍物
        for i = 1:size(circleCenter,1)
            if norm(point - circleCenter(i,:)) <= r(i)
                feasible = false; return;
            end
        end
        % 检查矩形障碍物
        for i = 1:size(cuboids,1)
            if all(point >= cuboids(i,1:3) & point <= cuboids(i,1:3) + cuboids(i,4:6))
                feasible = false; return;
            end
        end
    end
end
%% 辅助函数：判断三个点是否在同一条直线上
function isCollinear = isCollinear(p1, p2, p3)
    % 计算向量p1p2和p1p3的叉积
    v1 = p2 - p1;
    v2 = p3 - p1;
    crossProduct = cross(v1, v2);
    % 如果叉积为零向量，则三点共线
    isCollinear = all(abs(crossProduct) < 1e-6); % 允许一个小的容差
end


function drawCuboid(cuboid, color)
    [x, y, z] = ndgrid([cuboid(1), cuboid(1) + cuboid(4)], ...
                       [cuboid(2), cuboid(2) + cuboid(5)], ...
                       [cuboid(3), cuboid(3) + cuboid(6)]);
    x = x(:);
    y = y(:);
    z = z(:);
    k = convhull(x, y, z);
    trisurf(k, x, y, z, 'FaceColor', color, 'EdgeColor', 'w','LineWidth', 1.5);
end

