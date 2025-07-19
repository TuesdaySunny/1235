clc;
clear;
close all;
rng('shuffle'); % <-- 添加这一行

degtorad = pi/180;

d1 = 96;   
d2 = 0; 
d3 = 0;    
d4 = 122;    
d5 = 98;   
d6 = 89;   
d = [d1 d2 d3 d4 d5 d6]; % 

a1 = 0; 
a2 = 418; % 
a3 = 398; % 
a4 = 0;
a5 = 0;
a6 = 0;
a = [a1 a2 a3 a4 a5 a6]; % 

alpha1 = 90 * degtorad;  % 
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


L(1) = Link([theta1 d1 a1 alpha1], 'standard'); 
L(2) = Link([theta2 d2 a2 alpha2], 'standard'); 
L(3) = Link([theta3 d3 a3 alpha3], 'standard'); 
L(4) = Link([theta4 d4 a4 alpha4], 'standard'); 
L(5) = Link([theta5 d5 a5 alpha5], 'standard');
L(6) = Link([theta6 d6 a6 alpha6], 'standard');

% 
robot = SerialLink(L, 'name', 'EC 66');
% robot.teach(); % 启动 teach ，允许交互式调整关节角度
hold on;

% 
% q_start = [-pi/4 0 pi/6 -pi pi/2 0];
% 
% q_goal  = [-pi -pi/10 pi/9 0 -pi/2 0];

q_start = [pi*0.6422 pi*0.7207 pi*0.2424 -pi*1.1379 pi*0.4071 -pi*0.9445];

q_goal  = [-pi*0.09649 pi*0.9913 pi*0.1118 -pi*1.1147 pi*0.4036 0];



T_start = zeros(4, 4); % 初始化起始位姿矩阵
T_goal = zeros(4, 4);  % 初始化目标位姿矩阵
T_start(:, :) = robot.fkine(q_start); % 计算起始位姿
T_goal(:, :) = robot.fkine(q_goal);   % 计算目标位姿

% 在三维空间中绘制起始点和目标点
figure(1);
plot3(472, -645, 424, '-o', 'Color', 'r', 'MarkerSize', 15, 'MarkerFaceColor', 'r'); % 绘制目标点
hold on;
plot3(-884, 119, -112, '-o', 'Color', 'g', 'MarkerSize', 15, 'MarkerFaceColor', 'b'); % 绘制起始点
hold on;

q_min = [-2*pi -2*pi -2*pi -2*pi -2*pi -2*pi]; % 最小角度
q_max = [ 2*pi  2*pi  2*pi  2*pi  2*pi  2*pi]; % 最大角度
% 定义每个连杆的半径（用于碰撞检测）
link_radius = 50;


sphere_center = [
    -350  0 -250;  % 第一个球体的中心
       -600 -700  -600; % 第二个球体的中心
    
      450 -280 650;% 第四个球体的中心
      -750 220 450;
      -250 0 900;
      500 -750 -100;
      800 800 800
];
sphere_radius = [200, 240, 150,200,200,250,300]; % 每个球体的半径


cuboids = [ -500, 300,700, 600, 500, 400;-200,-800,-400,400,300,600;-900,600,-900,400,400,550];


draw_sphere(sphere_center, sphere_radius, 7);

[xSphere, ySphere, zSphere] = sphere;
for i = 1:size(cuboids, 1)
    drawCuboid(cuboids(i, :), 'k');
end
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal; 
hold on;
view(3);




xlim([-1200, 1200]);
ylim([-1200, 1200]);
zlim([-1200, 1200]);

%% 使用 RRT 算法规划路径
%检查是否有已保存的路径。如果需要重新规划路径，请取消注释以下三行
% [path, path_found] = RRT8(robot, q_min, q_max, q_start, q_goal, link_radius, ...
%     sphere_center, sphere_radius, cuboids);
% save('path.mat', 'path');

% 加载已保存的路径文件


 load('path.mat');
path_found = true;

if path_found
    T = zeros(4, 4, length(path.pos)); % 初始化路径位姿矩阵
    for i = 1:length(path.pos)
        T(:, :, i) = robot.fkine(path.pos(i).q); % 通过正向运动学计算路径中的位姿
    end
 
    plot3(squeeze(T(1, 4, :)), squeeze(T(2, 4, :)), squeeze(T(3, 4, :)), 'g-', 'LineWidth', 4);
    hold on;

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
    R(4,:,:) = []; % 移除第 4 行
    R(:,4,:) = []; % 移除第 4 列
    
    rpy = [];

    for i = 1:length(R)
    
        rpy = [rpy; rotm2rpy(R(:,:,i))];
    end
    rpy = rpy'; % 转置 RPY 矩阵，方便后续操作

    rpy = [rpy(:,1) rpy(:,1) rpy rpy(:,end) rpy(:,end)];    

    inter_rpy_b_spline = flip(b_spline(rpy, t, k), 2);

    inter_R_b_spline = zeros(3,3,length(inter_rpy_b_spline));
    for i = 1:length(inter_R_b_spline)
        % rpy2rotm 将平滑后的欧拉角转换为旋转矩阵
        inter_R_b_spline(:,:,i) = rpy2rotm(inter_rpy_b_spline(:,i));
    end

    inter_T_b_spline = zeros(4,4,length(inter_rpy_b_spline));
    for i = 1:length(inter_R_b_spline)
        % 将旋转矩阵和位置数据组合为完整的 4x4 位姿矩阵
        inter_T_b_spline(1:3,1:4,i) = [inter_R_b_spline(:,:,i) inter_path_b_spline(:,i)];
        inter_T_b_spline(4,:,i) = [0 0 0 1]; % 最后一行保持为齐次矩阵的形式 [0 0 0 1]
    end

    theta = zeros(8,6,length(inter_T_b_spline));
    for i = 1:length(inter_T_b_spline)

        theta(:,:,i) = niyundongxue(inter_T_b_spline(:,:,i), a, d);
    end
% ------- 原有的逆解求解结果 theta(j,:,i-1) 已经存在 --------

% 初步选择角度（用你原来的方法或我之前帮你重构的那个）
theta_choose = zeros(size(theta,3), 6);
theta_choose(1, :) = q_start; % 起点角度

angle_jump_thresh = deg2rad(30); % 跳变阈值

for i = 2:size(theta,3)+1
    min_error = inf;
    best_q = [];
    found_continuous = false;

    for j = 1:8
        q_candidate = theta(j,:,i-1);
        angle_diff = abs(q_candidate - theta_choose(i-1,:));

        if all(angle_diff < angle_jump_thresh)
            error = norm(angle_diff);
            if error < min_error
                min_error = error;
                best_q = q_candidate;
                found_continuous = true;
            end
        end
    end

    if found_continuous
        theta_choose(i,:) = best_q;
    else
        % fallback
        min_error = inf;
        for j = 1:8
            q_candidate = theta(j,:,i-1);
            error = norm(q_candidate - theta_choose(i-1,:));
            if error < min_error
                min_error = error;
                best_q = q_candidate;
            end
        end
        theta_choose(i,:) = best_q;
    end
end

% -------- 改为对角度做插值，而不是末端轨迹 --------
fps = 20;
numPoints = size(theta_choose,1);
totalTime = (numPoints-1) / fps;

% 时间参数
t_original = linspace(0, 1, numPoints);  % 原时间分布
Pr = 200;  % 插值后轨迹点数
t_interp = linspace(0, 1, Pr);           % 插值时间分布

theta_interp = zeros(Pr, 6);
for i = 1:6
    theta_interp(:,i) = interp1(t_original, theta_choose(:,i), t_interp, 'spline');
end

% --------- 动画演示 ---------
robot.plot(theta_interp, 'fps', 20);

% --------- 绘制角度曲线 ---------
timeVec = linspace(0, totalTime, Pr);

figure(2); 
hold on;
for jointIdx = 1:6
    plot(timeVec, rad2deg(theta_interp(:, jointIdx)), 'LineWidth', 2, ...
         'DisplayName', ['Joint ', num2str(jointIdx)]);
end
xlabel('Time (s)');
ylabel('Joint Angle (deg)');
title('All Joint Angles vs. Time (Smoothed)');
legend('Location','best'); 
grid on;

% --------- 绘制角度曲线（平滑版） ---------
figure(3); 
hold on;

% 使用 smoothdata 进行曲线平滑处理
theta_smooth = smoothdata(theta_interp, 1, 'movmean', 10); % 可调窗口大小

for jointIdx = 1:6
    plot(timeVec, rad2deg(theta_smooth(:, jointIdx)), 'LineWidth', 2, ...
         'DisplayName', ['Joint ', num2str(jointIdx)]);
end

xlabel('Time (s)');
ylabel('Joint Angle (deg)');
title('All Joint Angles vs. Time (Smoothed & Filtered)');
legend('Location','best'); 
grid on;


else
    disp('No path found.');
end

function S = b_spline(Poses,t,k)
    S = [];
    n = size(Poses,2) - 1;
    if (k==2)

        disp('Using Quadratic B-spline')
        M = 0.5.*[1 -2 1;-2 2 1;1 0 0]; % Basis function matrix
        T = [t.^2;t.^1;t.^0];
        for i = 1:n-k+1
            B = Poses(:,i:i+k)*M*T; % B-spline segment
            S = [S B];
        end
        return

    elseif (k==3)
        disp('Using Cubic B-spline')
        M = (1/6).*[-1 3 -3 1;3 -6 3 0;-3 0 3 0;1 4 1 0]; % Basis function matrix
        T = [t.^3;t.^2;t.^1;t.^0]';
        for i = 1:n-k+1
            B = T*M*Poses(:,i:i+k)'; % B-spline segment
            S = [S;B];
        end      
        S = S';
        return

    else
        disp('This function works with quadratic and cubic B-splines!')
        return 
    end    
end

function draw_sphere(position, radius, num)
    % 遍历每个球体，计算并绘制其表面
    for i = 1:num
        % 生成单位球体的坐标 (x, y, z)
        [s(i).X, s(i).Y, s(i).Z] = sphere;
        
        % 按指定半径缩放球体
        % 将单位球的坐标乘以当前球体的半径，得到缩放后的球体坐标
        s(i).X = s(i).X * radius(i);
        s(i).Y = s(i).Y * radius(i);
        s(i).Z = s(i).Z * radius(i);
        
        % 平移球体到指定的位置
        % 将球体的坐标加上当前球体中心的 x、y、z 坐标
        s(i).X = s(i).X + position(i,1); % 平移 x 坐标
        s(i).Y = s(i).Y + position(i,2); % 平移 y 坐标
        s(i).Z = s(i).Z + position(i,3); % 平移 z 坐标
        
%         % 绘制球体的表面
%         surf(s(i).X, s(i).Y, s(i).Z); % 使用 surf 函数绘制球体
colors = ['b', 'r', 'm', 'm', 'c', 'y', 'r', 'w','m','b']; 

surf(s(i).X, s(i).Y, s(i).Z, 'FaceColor', colors(mod(i-1, length(colors))+1)); % 循环使用颜色


    end
end


function drawCuboid(cuboid, color)
    [x, y, z] = ndgrid([cuboid(1), cuboid(1) + cuboid(4)], ...
                       [cuboid(2), cuboid(2) + cuboid(5)], ...
                       [cuboid(3), cuboid(3) + cuboid(6)]);
    x = x(:);
    y = y(:);
    z = z(:);
    k = convhull(x, y, z);
    trisurf(k, x, y, z, 'FaceColor', color, 'EdgeColor', 'w', 'LineWidth', 1.5);
    hold on;
end

function theta=niyundongxue(T, a, d)
    % 变换矩阵T已知
    % SDH:标准DH参数表求逆解（解析解）
    % 部分DH参数表如下，需要求解theta信息
    % thanks to https://blog.csdn.net/weixin_40534238/article/details/89378744
    alpha=[pi/2,0,0,pi/2,-pi/2,0];% alpha没有用到,故此逆解程序只适合alpha=[pi/2,0,0,pi/2,-pi/2,0]的情况！
    
    nx=T(1,1);ny=T(2,1);nz=T(3,1);
    ox=T(1,2);oy=T(2,2);oz=T(3,2);
    ax=T(1,3);ay=T(2,3);az=T(3,3);
    px=T(1,4);py=T(2,4);pz=T(3,4);
    
    %求解关节角1
    m=d(6)*ay-py;  n=ax*d(6)-px; 
    theta1(1,1)=atan2(m,n)-atan2(d(4),sqrt(m^2+n^2-(d(4))^2));
    theta1(1,2)=atan2(m,n)-atan2(d(4),-sqrt(m^2+n^2-(d(4))^2));
  
    %求解关节角5
    theta5(1,1:2)=acos(ax*sin(theta1)-ay*cos(theta1));
    theta5(2,1:2)=-acos(ax*sin(theta1)-ay*cos(theta1));      
    
    %求解关节角6
    mm=nx*sin(theta1)-ny*cos(theta1); nn=ox*sin(theta1)-oy*cos(theta1);
    theta6=atan2(mm,nn)-atan2(sin(theta5),0);
    
    %求解关节角3
    mmm=d(5)*(sin(theta6).*(nx*cos(theta1)+ny*sin(theta1))+cos(theta6).*(ox*cos(theta1)+oy*sin(theta1))) ...
        -d(6)*(ax*cos(theta1)+ay*sin(theta1))+px*cos(theta1)+py*sin(theta1);
    nnn=pz-d(1)-az*d(6)+d(5)*(oz*cos(theta6)+nz*sin(theta6));
    theta3(1:2,:)=real(acos((mmm.^2+nnn.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3))));
    theta3(3:4,:)=real(-acos((mmm.^2+nnn.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3))));
    
    %求解关节角2
    mmm_s2(1:2,:)=mmm;
    mmm_s2(3:4,:)=mmm;
    nnn_s2(1:2,:)=nnn;
    nnn_s2(3:4,:)=nnn;
    s2=((a(3)*cos(theta3)+a(2)).*nnn_s2-a(3)*sin(theta3).*mmm_s2)./ ...
        ((a(2))^2+(a(3))^2+2*a(2)*a(3)*cos(theta3));
    c2=(mmm_s2+a(3)*sin(theta3).*s2)./(a(3)*cos(theta3)+a(2));
    theta2=atan2(s2,c2);   
    
    %整理关节角1 5 6 3 2
    theta(1:4,1)=theta1(1,1);theta(5:8,1)=theta1(1,2);
    theta(:,2)=[theta2(1,1),theta2(3,1),theta2(2,1),theta2(4,1),theta2(1,2),theta2(3,2),theta2(2,2),theta2(4,2)]';
    theta(:,3)=[theta3(1,1),theta3(3,1),theta3(2,1),theta3(4,1),theta3(1,2),theta3(3,2),theta3(2,2),theta3(4,2)]';
    theta(1:2,5)=theta5(1,1);theta(3:4,5)=theta5(2,1);
    theta(5:6,5)=theta5(1,2);theta(7:8,5)=theta5(2,2);
    theta(1:2,6)=theta6(1,1);theta(3:4,6)=theta6(2,1);
    theta(5:6,6)=theta6(1,2);theta(7:8,6)=theta6(2,2); 
    
    %求解关节角4
    theta(:,4)=atan2(-sin(theta(:,6)).*(nx*cos(theta(:,1))+ny*sin(theta(:,1)))-cos(theta(:,6)).* ...
        (ox*cos(theta(:,1))+oy*sin(theta(:,1))),oz*cos(theta(:,6))+nz*sin(theta(:,6)))-theta(:,2)-theta(:,3);  
    
end

function rpy = rotm2rpy( R )

if abs(R(3 ,1) - 1.0) < 1.0e-15   % singularity
    a = 0.0;
    b = -pi / 2.0;
    c = atan2(-R(1, 2), -R(1, 3));
elseif abs(R(3, 1) + 1.0) < 1.0e-15   % singularity
    a = 0.0;
    b = pi / 2.0;
    c = -atan2(R(1, 2), R(1, 3));
else
    a = atan2(R(3, 2), R(3, 3));
    c = atan2(R(2, 1), R(1, 1));
    %     a = atan2(-R(3, 2), -R(3, 3));  %a另一个解
    %     c = atan2(-R(2, 1), -R(1, 1));  %c另一个解
    cosC = cos(c);
    sinC = sin(c);
    
    if abs(cosC) > abs(sinC)
        b = atan2(-R(3, 1), R(1, 1) / cosC);
    else
        b = atan2(-R(3, 1), R(2, 1) / sinC);
    end
end

rpy = [a, b, c];

end

function R = rpy2rotm(rpy)

a = rpy(1);
b = rpy(2);
c = rpy(3);

sinA = sin(a);
cosA = cos(a);
sinB = sin(b);
cosB = cos(b);
sinC = sin(c);
cosC = cos(c);

R = [cosB*cosC,  cosC*sinA*sinB - cosA*sinC,  sinA*sinC + cosA*cosC*sinB
    cosB*sinC, cosA*cosC + sinA*sinB*sinC, cosA*sinB*sinC - cosC*sinA
    -sinB, cosB*sinA,  cosA*cosB];

end




