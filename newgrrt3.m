close all;
clear all;
%% 绘制障碍物(以球为例，主要是方便计算)
x0=100; y0=100; z0=100;%球心
circleCenter = [100,100,100;50,80,50;100,40,60;150,100,100;60,130,50];
r=[20;20;20;15;15];%半径
%下面开始画
figure(1);
[x,y,z]=sphere;
for i = 1:length(circleCenter(:,1))
    mesh(r(i)*x+circleCenter(i,1),r(i)*y+circleCenter(i,2),r(i)*z+circleCenter(i,3));hold on;
end
axis equal

%% 参数
source=[10 10 10];
goal=[150 150 150];
stepsize = 10;
threshold = 10;
maxFailedAttempts = 10000;
display = true;
searchSize = [250 250 250];      %探索空间六面体

%% 绘制起点和终点
hold on;
scatter3(source(1),source(2),source(3),"filled","g");
scatter3(goal(1),goal(2),goal(3),"filled","b");

tic;  % tic-toc: Functions for Elapsed Time
RRTree = double([source -1]);
failedAttempts = 0;
pathFound = false;

%% 循环
while failedAttempts <= maxFailedAttempts  % loop to grow RRTs
    %% chooses a random configuration
    if rand < 0.5
        sample = rand(1,3) .* searchSize;   % random sample
    else
        sample = goal; % sample taken as goal to bias tree generation to goal
    end
    %% selects the node in the RRT tree that is closest to qrand
    [A, I] = min( distanceCost(RRTree(:,1:3),sample) ,[],1); % find the minimum value of each column
    closestNode = RRTree(I(1),1:3);
    %% moving from qnearest an incremental distance in the direction of qrand
    movingVec = [sample(1)-closestNode(1),sample(2)-closestNode(2),sample(3)-closestNode(3)];
    movingVec = movingVec/sqrt(sum(movingVec.^2));  %单位化
    
    newPoint = closestNode + stepsize * movingVec;
    if ~checkPath3(closestNode, newPoint, circleCenter,r) % if extension of closest node in tree to the new point is feasible
        failedAttempts = failedAttempts + 1;
        continue;
    end
    
    if distanceCost(newPoint,goal) < threshold, pathFound = true; break; end % goal reached
    [A, I2] = min( distanceCost(RRTree(:,1:3),newPoint) ,[],1); % check if new node is not already pre-existing in the tree
    if distanceCost(newPoint,RRTree(I2(1),1:3)) < threshold, failedAttempts = failedAttempts + 1; continue; end 
    
    RRTree = [RRTree; newPoint I(1)]; % add node
    failedAttempts = 0;
    if display, plot3([closestNode(1);newPoint(1)],[closestNode(2);newPoint(2)],[closestNode(3);newPoint(3)],'LineWidth',1); end
    pause(0.05);
end

if display && pathFound, plot3([closestNode(1);goal(1)],[closestNode(2);goal(2)],[closestNode(3);goal(3)]); end

if display, disp('click/press any key'); waitforbuttonpress; end
if ~pathFound, error('no path found. maximum attempts reached'); end

%% retrieve path from parent information
path = goal;
prev = I(1);
while prev > 0
    path = [RRTree(prev,1:3); path];
    prev = RRTree(prev,4);
end

pathLength = 0;
for i=1:length(path(:,1))-1, pathLength = pathLength + distanceCost(path(i,1:3),path(i+1,1:3)); end % calculate path length
fprintf('processing time=%d \nPath Length=%d \n\n', toc, pathLength); 
figure(2)
for i = 1:length(circleCenter(:,1))
    mesh(r(i)*x+circleCenter(i,1),r(i)*y+circleCenter(i,2),r(i)*z+circleCenter(i,3));hold on;
end
axis equal
hold on;
scatter3(source(1),source(2),source(3),"filled","g");
scatter3(goal(1),goal(2),goal(3),"filled","b");
plot3(path(:,1),path(:,2),path(:,3),'LineWidth',2,'color','r');
% 初始化每个障碍物到路径的最小距离数组
minDistancesToPath = inf(length(circleCenter(:,1)), 1);
% 初始化每个障碍物最近路径点的索引
closestPathPointIndices = zeros(length(circleCenter(:,1)), 1);

% 遍历所有的障碍物
for i = 1:length(circleCenter(:,1))
    % 遍历路径中的所有点
    for j = 1:length(path(:,1))
        point = path(j, :); % 当前路径点
        % 计算当前路径点到障碍物中心的距离
        d = norm(point - circleCenter(i,:));
        % 计算当前路径点到障碍物表面的距离
        distanceToSurface = abs(d - r(i));
        
        % 如果这是到当前障碍物最近的路径点，则更新最小距离和索引
        if distanceToSurface < minDistancesToPath(i)
            minDistancesToPath(i) = distanceToSurface;
            closestPathPointIndices(i) = j;
        end
    end
    
    % 在路径和障碍物之间绘制最短距离线段
    closestPointOnPath = path(closestPathPointIndices(i), :);
    % 计算最近路径点在障碍物表面上的对应点
    directionVec = (closestPointOnPath - circleCenter(i, :)) / norm(closestPointOnPath - circleCenter(i, :));
    closestPointOnSurface = circleCenter(i, :) + directionVec * r(i);
    
    % 绘制线段
    plot3([closestPointOnPath(1), closestPointOnSurface(1)], ...
          [closestPointOnPath(2), closestPointOnSurface(2)], ...
          [closestPointOnPath(3), closestPointOnSurface(3)], 'm-', 'LineWidth', 2);
    % 标注障碍物序号
    text(circleCenter(i, 1), circleCenter(i, 2), circleCenter(i, 3), ...
         ['Obstacle ', num2str(i)], 'VerticalAlignment', 'bottom', ...
         'HorizontalAlignment', 'right');
end

% 显示每个障碍物到最近路径的最小距离
for i = 1:length(minDistancesToPath)
    disp(['障碍物 ', num2str(i), ' 到最近路径的最小距离: ', num2str(minDistancesToPath(i))]);
    % 在图中标注最小距离的值
    closestPointOnPath = path(closestPathPointIndices(i), :);
    text(mean([closestPointOnPath(1), circleCenter(i, 1)]), ...
         mean([closestPointOnPath(2), circleCenter(i, 2)]), ...
         mean([closestPointOnPath(3), circleCenter(i, 3)]) + 5, ...
         ['d_{min} = ', num2str(minDistancesToPath(i), '%.2f')], ...
         'Color', 'm');
end

%% checkPath3.m	
function feasible=checkPath3(n,newPos,circleCenter,r)
feasible=true;
movingVec = [newPos(1)-n(1),newPos(2)-n(2),newPos(3)-n(3)];
movingVec = movingVec/sqrt(sum(movingVec.^2)); %单位化
for R=0:0.5:sqrt(sum((n-newPos).^2))
    posCheck=n + R .* movingVec;
    if ~(feasiblePoint3(ceil(posCheck),circleCenter,r) && feasiblePoint3(floor(posCheck),circleCenter,r))
        feasible=false;break;
    end
end
if ~feasiblePoint3(newPos,circleCenter,r), feasible=false; end
end


%% feasiblePoint3.m
function feasible=feasiblePoint3(point,circleCenter,r)
feasible=true;
% check if collission-free spot and inside maps
for row = 1:length(circleCenter(:,1))
    if sqrt(sum((circleCenter(row,:)-point).^2)) <= r(row)+2
        feasible = false;break;
    end
end
end




