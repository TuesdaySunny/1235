clc; 
clear; 
close all;

%% 创建机器人模型
% 将角度从度数转换为弧度
degtorad = pi/180;

% 设置每个连杆的偏移距离（D-H 参数中的 d 值，单位：毫米）
d1 = 96;   % 第一个连杆的偏移距离
d2 = 0; % 第二个连杆的偏移距离
d3 = 0;     % 第三个连杆的偏移距离
d4 = 122;     % 第四个连杆的偏移距离
d5 = 98;   % 第五个连杆的偏移距离
d6 = 89;   % 第六个连杆的偏移距离
d = [d1 d2 d3 d4 d5 d6]; % 将偏移值存储为一个数组

% 设置每个连杆的长度（D-H 参数中的 a 值，单位：毫米）
a1 = 0; 
a2 = 418; % 第二个连杆的长度
a3 = 398; % 第三个连杆的长度
a4 = 0;
a5 = 0;
a6 = 0;
a = [a1 a2 a3 a4 a5 a6]; % 将连杆长度存储为一个数组

% 设置每个连杆的扭转角（D-H 参数中的 alpha 值，单位：弧度）
alpha1 = 90 * degtorad;  % 第一个连杆的扭转角
alpha2 = 0* degtorad;
alpha3 = 0* degtorad;
alpha4 = 90 * degtorad;
alpha5 = -90 * degtorad;
alpha6 = 0 * degtorad;

theta1 =90 * degtorad;
theta2 =90 * degtorad;
theta3 =0 * degtorad;
theta4 =90 * degtorad;
theta5 =0 * degtorad;
theta6 =0 * degtorad;


% 使用标准型 D-H 参数创建连杆对象，并存储到 L 数组中
L(1) = Link([theta1 d1 a1 alpha1], 'standard'); 
L(2) = Link([theta2 d2 a2 alpha2], 'standard'); 
L(3) = Link([theta3 d3 a3 alpha3], 'standard'); 
L(4) = Link([theta4 d4 a4 alpha4], 'standard'); 
L(5) = Link([theta5 d5 a5 alpha5], 'standard');
L(6) = Link([theta6 d6 a6 alpha6], 'standard');

% 创建一个六自由度的机器人模型
robot = SerialLink(L, 'name', 'EC 66');
% robot.teach(); % 启动 teach 界面，允许交互式调整关节角度
hold on;

% 起始和目标配置
% 定义起始关节角度
q_start = [-pi/4 0 pi/6 -pi pi/2 0];
% 定义目标关节角度
q_goal  = [-pi -pi/10 pi/9 0 -pi/2 0];
% % 定义起始关节角度
% q_start = [pi*10/180 -pi/6 -pi/6 -pi/2 0 0];
% % 定义目标关节角度
% q_goal  = [pi pi/3 -pi/3 0 0 0];

% 通过正向运动学计算起始和目标位姿矩阵
T_start = zeros(4, 4); % 初始化起始位姿矩阵
T_goal = zeros(4, 4);  % 初始化目标位姿矩阵
T_start(:, :) = robot.fkine(q_start); % 计算起始位姿
T_goal(:, :) = robot.fkine(q_goal);   % 计算目标位姿

% 在三维空间中绘制起始点和目标点
figure(1);
plot3(T_goal(1, 4), T_goal(2, 4), T_goal(3, 4), '-o', 'Color', 'r', 'MarkerSize', 15, 'MarkerFaceColor', 'r'); % 绘制目标点
hold on;
plot3(T_start(1, 4), T_start(2, 4), T_start(3, 4), '-o', 'Color', 'g', 'MarkerSize', 15, 'MarkerFaceColor', 'b'); % 绘制起始点
hold on;

% 设置每个关节的最小和最大角度限制
% q_min = [-pi -pi -pi -pi -pi -pi]; % 最小角度
% q_max = [ pi  pi  pi  pi  pi  pi]; % 最大角度
q_min = [-2*pi -2*pi -2*pi -2*pi -2*pi -2*pi]; % 最小角度
q_max = [ 2*pi  2*pi  2*pi  2*pi  2*pi  2*pi]; % 最大角度
% 定义每个连杆的半径（用于碰撞检测）
link_radius = 50;


% 定义场景中的球体障碍物
sphere_center = [
    -350  0 -250;  % 第一个球体的中心
       -600 -700  -600; % 第二个球体的中心
       % 第三个球体的中心
      450 -280 650;% 第四个球体的中心
      -750 220 450;
      -250 0 900;
      500 -750 -100;
      800 800 800
];
sphere_radius = [200, 240, 150,200,200,250,300]; % 每个球体的半径


cuboids = [ -500, 300,700, 600, 500, 400;-200,-800,-400,400,300,600;-900,600,-900,400,400,550];

% 绘制球体障碍物
draw_sphere(sphere_center, sphere_radius, 7);

% 绘制立方体障碍物
% draw_cuboid(cuboid_origin, cuboid_ckg, 2);

% cuboids = [originPoint, cuboidSize];  % originPoint 为 [x, y, z]，cuboidSize 为 [width, height, depth]
[xSphere, ySphere, zSphere] = sphere;
for i = 1:size(cuboids, 1)
    drawCuboid(cuboids(i, :), 'k');
end
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; 
hold on;
view(3);







% % 定义场景中的球体障碍物
% sphere_center = [
%     -350  0 -250;  % 第一个球体的中心
%        -600 -700  -600; % 第二个球体的中心
%        % 第三个球体的中心
%       300 -280 650;% 第四个球体的中心
%       -750 220 350;
%       -300 -100 850;
%       500 -750 -100;
%       800 800 800
% ];
% sphere_radius = [200, 240, 150,200,200,250,300]; % 每个球体的半径
% 
% % 定义场景中的立方体障碍物
% % cuboid_origin = 4*[
% %     
% %     100 -100 0; 
% %     130 -70  150
% % ];
% % cuboid_ckg = 4*[
% %     
% %     80  20 100; 
% %     100 150 150
% % ];
% cuboids = [ -500, 300,700, 600, 500, 400;-200,-800,-400,400,300,600;-900,600,-900,400,400,550];
% 
% % 绘制球体障碍物
% draw_sphere(sphere_center, sphere_radius, 7);
% 
% % 绘制立方体障碍物
% % draw_cuboid(cuboid_origin, cuboid_ckg, 2);
% 
% % cuboids = [originPoint, cuboidSize];  % originPoint 为 [x, y, z]，cuboidSize 为 [width, height, depth]
% [xSphere, ySphere, zSphere] = sphere;
% for i = 1:size(cuboids, 1)
%     drawCuboid(cuboids(i, :), 'k');
% end
% xlabel('X'); ylabel('Y'); zlabel('Z');
% axis equal; 
% hold on;
% view(3);





% 设置三维绘图的范围
xlim([-1200, 1200]);
ylim([-1200, 1200]);
zlim([-1200, 1200]);

%% 使用 RRT 算法规划路径
%检查是否有已保存的路径。如果需要重新规划路径，请取消注释以下三行
% [path, path_found] = RRT8(robot, q_min, q_max, q_start, q_goal, link_radius, ...
%     sphere_center, sphere_radius, cuboid_origin, cuboid_ckg);
% save('path.mat', 'path');

% 加载已保存的路径文件
 load('path.mat');
path_found = true;

% 如果找到路径，则绘制路径
if path_found
    T = zeros(4, 4, length(path.pos)); % 初始化路径位姿矩阵
    for i = 1:length(path.pos)
        T(:, :, i) = robot.fkine(path.pos(i).q); % 通过正向运动学计算路径中的位姿
    end
    % 绘制路径的三维轨迹
    plot3(squeeze(T(1, 4, :)), squeeze(T(2, 4, :)), squeeze(T(3, 4, :)), 'g-', 'LineWidth', 4);
    hold on;

%     %% smooth
%     % 如果找到轨迹，则对轨迹进行平滑处理
%     % SmoothenPath 是一个自定义函数，用于对路径进行平滑优化。
%     % 输入：机器人模型（robot）、路径点（path.pos）、连杆半径（link_radius）、
%     %       球体障碍物参数（sphere_center 和 sphere_radius）、
%     %       立方体障碍物参数（cuboid_origin 和 cuboid_ckg）。
%     % 输出：平滑后的路径（smoothed_path）。
%     smoothed_path = SmoothenPath(robot, path.pos, link_radius, sphere_center, sphere_radius, cuboid_origin, cuboid_ckg);
%     
%     % 将平滑后的路径可视化
%     % 打印平滑路径的中间节点数量（去掉起点和终点）
%     fprintf('Smoothed path found with %d intermediate waypoints:\n', size(smoothed_path, 1) - 2);
%     % 显示平滑路径的具体点坐标
%     disp(smoothed_path);
%     
%     % 使用 flip 函数对路径进行翻转（倒序排列路径点）
%     % interpolate_path 函数用于对路径进行插值，生成更密集的路径点
%     smoothed_path_inter = flip(interpolate_path(smoothed_path), 1);
%     % 打印插值后的路径点
%     fprintf('smoothed path after interpolating :\n');
%     disp(smoothed_path_inter);
% 
%     % 初始化用于存储插值路径点位姿的矩阵 T2
%     % T2 的维度为 4x4xN，其中 N 是插值路径点的数量
%     T2 = zeros(4,4,size(smoothed_path_inter,1));
%     for i = 1:size(smoothed_path_inter,1)
%         % 通过正向运动学计算每个插值路径点的位姿矩阵
%         T2(:,:,i) = robot.fkine(smoothed_path_inter(i,:));
%     end
%     % 绘制插值平滑后的路径的三维轨迹
%     % squeeze 函数用于去除矩阵的多余维度，提取 x、y、z 的位置坐标
%     plot3(squeeze(T2(1, 4, :)), squeeze(T2(2, 4, :)), squeeze(T2(3, 4, :)), 'b-', 'LineWidth', 2);
%     
%     % 将插值平滑后的路径以动画形式播放
%     % robot.plot 函数会依次显示路径点对应的关节角度姿态
%     % 'fps' 参数设置动画的帧率
%     robot.plot(smoothed_path_inter, 'fps', 10);
% RRT


    % 使用 B 样条曲线平滑路径
    k = 3; % 三次 B 样条
    Pr = 50; % 样条曲线的点数
    t = linspace(0, 1, Pr); % 参数化变量
    pos_path = [squeeze(T(1, 4, :)), squeeze(T(2, 4, :)), squeeze(T(3, 4, :))].';
    pos_path = [pos_path(:, 1) pos_path(:, 1) pos_path pos_path(:, end) pos_path(:, end)];
    inter_path_b_spline = flip(b_spline(pos_path, t, k), 2);
    plot3(inter_path_b_spline(1, :), inter_path_b_spline(2, :), inter_path_b_spline(3, :), ...
        'r-', 'LineWidth', 4, 'DisplayName', 'Cubic-B-spline');
    R = T; 
    % 从 T 中移除最后一行，保留 3x3 的旋转矩阵部分
    R(4,:,:) = []; % 移除第 4 行
    R(:,4,:) = []; % 移除第 4 列
    
    rpy = [];
    % 将旋转矩阵 R 转换为欧拉角（Roll-Pitch-Yaw）
    for i = 1:length(R)
        % rotm2rpy 将旋转矩阵转换为对应的 RPY 欧拉角
        rpy = [rpy; rotm2rpy(R(:,:,i))];
    end
    rpy = rpy'; % 转置 RPY 矩阵，方便后续操作

    % 在 RPY 数据的起点和终点增加冗余控制点，方便 B 样条平滑
    % 冗余点的增加可以改善样条曲线的收敛效果
    rpy = [rpy(:,1) rpy(:,1) rpy rpy(:,end) rpy(:,end)];    

    % 使用 B 样条对 RPY 数据进行平滑
    % flip 函数将 B 样条结果倒序排列，保证路径起点在最前
    inter_rpy_b_spline = flip(b_spline(rpy, t, k), 2);

    % 初始化平滑后的旋转矩阵序列
    % inter_R_b_spline 存储平滑后的位置对应的旋转矩阵
    inter_R_b_spline = zeros(3,3,length(inter_rpy_b_spline));
    for i = 1:length(inter_R_b_spline)
        % rpy2rotm 将平滑后的欧拉角转换为旋转矩阵
        inter_R_b_spline(:,:,i) = rpy2rotm(inter_rpy_b_spline(:,i));
    end

    % 初始化平滑后的位姿矩阵序列
    % inter_T_b_spline 包含了 4x4 的位姿矩阵
    inter_T_b_spline = zeros(4,4,length(inter_rpy_b_spline));
    for i = 1:length(inter_R_b_spline)
        % 将旋转矩阵和位置数据组合为完整的 4x4 位姿矩阵
        inter_T_b_spline(1:3,1:4,i) = [inter_R_b_spline(:,:,i) inter_path_b_spline(:,i)];
        inter_T_b_spline(4,:,i) = [0 0 0 1]; % 最后一行保持为齐次矩阵的形式 [0 0 0 1]
    end

    % 初始化逆运动学解的存储矩阵
    % theta 存储 8 组可能的逆运动学解，每组有 6 个关节角度
    theta = zeros(8,6,length(inter_T_b_spline));
    for i = 1:length(inter_T_b_spline)
        % 调用 niyundongxue 函数（需自定义），计算逆运动学解
        % 输入为位姿矩阵 inter_T_b_spline，输出为 8 组解
        theta(:,:,i) = niyundongxue(inter_T_b_spline(:,:,i), a, d);
    end
    
    % 初始化选择的关节角度序列
    % theta_choose 用于存储从 8 组解中选择的最优解
    theta_choose = zeros(size(theta,3), 6);
    theta_choose(1, :) = q_start; % 设置起点的关节角度为初始解
    
    % 遍历平滑路径的每个点，选择最接近上一个点的解
    for i = 2:size(theta,3)+1
        e = 1000000; % 初始化一个很大的误差值，用于寻找最小误差的解
        for j = 1:8 % 遍历当前点的 8 组解
            % 计算当前解和上一个选择的解之间的欧几里得距离
            if norm(theta(j,:,i-1) - theta_choose(i-1,:)) < e
                % 如果找到更小的误差，则更新最优解和误差值
                e = theta(j,:,i-1) - theta_choose(i-1,:);
                theta_choose(i,:) = theta(j,:,i-1);
            end
        end
    end
    % 将目标点的关节角度添加到选择解序列中
   % theta_choose = [theta_choose; q_goal];
    
    % 使用选择的关节角度序列绘制机械臂的运动轨迹
    % 'fps' 参数指定动画帧率，单位为帧每秒
    robot.plot(theta_choose, 'fps', 10);

else
    disp('No path found.');
end
