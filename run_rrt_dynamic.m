% 运行带有动态可视化效果的标准RRT算法示例脚本
% 加载之前生成的迷宫地图，并应用RRT算法进行路径规划

% 清理工作区域
clear all;
close all;

% 关闭数据显示侧边栏，提高性能
set(0, 'DefaultFigureWindowStyle', 'normal'); % 确保使用标准窗口
set(0, 'DefaultFigureVisible', 'on');

% 加载迷宫地图
try
    load('maze_map.mat');
    fprintf('已加载maze_map.mat文件\n');
catch
    % 如果没有找到地图文件，则先运行迷宫生成器
    fprintf('未找到maze_map.mat文件，正在运行迷宫生成器...\n');
    run('maze_generator.m');
    load('maze_map.mat');
end

% 显示原始地图
h_fig1 = figure('Name', '迷宫地图');
set(h_fig1, 'NumberTitle', 'off'); % 隐藏figure编号
set(h_fig1, 'MenuBar', 'none'); % 隐藏菜单栏
set(h_fig1, 'ToolBar', 'none'); % 隐藏工具栏

colormap([1 1 1; 0 0 0]); % 白色表示通道，黑色表示墙
pcolor(maze);
axis equal;
axis off;
title('迷宫地图');

% 标记起点和终点
hold on;
plot(start_x, start_y, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(end_x, end_y, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
legend('起点', '终点', 'Location', 'northeastoutside');

% 设置RRT参数
max_iterations = 5000;  % 最大迭代次数
step_size = 3;         % 步长
goal_threshold = 2.0;   % 目标阈值

% 打印参数信息
fprintf('标准RRT算法参数:\n');
fprintf('最大迭代次数: %d\n', max_iterations);
fprintf('步长: %d\n', step_size);
fprintf('目标阈值: %.1f\n', goal_threshold);
fprintf('起点坐标: (%d, %d)\n', start_x, start_y);
fprintf('终点坐标: (%d, %d)\n', end_x, end_y);

% 询问用户是否需要录制视频
record_video = input('是否需要录制算法运行过程视频？(1-是/0-否): ');

% 如果要录制视频，询问倍速
video_speed = 1;
if record_video
    speed_options = [1, 2, 4, 8];
    user_choice = input('请选择视频倍速(1-正常速度, 2-2倍速, 3-4倍速, 4-8倍速): ');
    if user_choice >= 1 && user_choice <= 4
        video_speed = speed_options(user_choice);
    end
    fprintf('已选择视频 %d 倍速\n', video_speed);
end

% 提示用户开始动态规划过程
fprintf('\n按任意键开始RRT路径规划动态演示...\n');
pause;

% 如果需要录制视频，设置视频写入器
if record_video
    % 设置视频参数
    video_filename = 'rrt_dynamic_animation.mp4';
    fps = 20 * video_speed; % 调整帧率实现倍速效果
    quality = 90; % 视频质量 (1-100)
    
    % 创建视频写入器对象
    video_writer = VideoWriter(video_filename, 'MPEG-4');
    video_writer.FrameRate = fps;
    video_writer.Quality = quality;
    
    % 打开视频写入器
    open(video_writer);
    
    fprintf('视频录制已开始，将保存到文件: %s (倍速: %dx)\n', video_filename, video_speed);
end

% 调用RRT动态可视化函数并传入视频写入器参数
fprintf('正在运行RRT算法，请观察动态过程...\n');
tic; % 开始计时
if record_video
    [path, G] = rrt_dynamic(maze, [start_x, start_y], [end_x, end_y], max_iterations, step_size, goal_threshold, video_writer);
else
    [path, G] = rrt_dynamic(maze, [start_x, start_y], [end_x, end_y], max_iterations, step_size, goal_threshold);
end
elapsed_time = toc; % 结束计时

% 关闭视频写入器（如果已打开）
if record_video && exist('video_writer', 'var')
    try
        close(video_writer);
        fprintf('视频录制已完成，文件已保存为: %s\n', video_filename);
    catch
        % 如果视频写入器已经关闭或出现其他错误，不进行任何操作
    end
end

% 显示结果
if ~isempty(path)
    fprintf('\n成功找到路径！\n');
    fprintf('路径长度: %d 个节点\n', size(path, 1));
    fprintf('RRT树节点总数: %d\n', size(G.nodes, 1));
    fprintf('算法运行时间: %.2f 秒\n', elapsed_time);
    
    % 显示路径长度（欧氏距离）
    path_length = 0;
    for i = 1:(size(path,1)-1)
        path_length = path_length + sqrt(sum((path(i,:) - path(i+1,:)).^2));
    end
    fprintf('路径欧氏距离: %.2f\n', path_length);
else
    fprintf('\n未找到可行路径！\n');
    fprintf('RRT树节点总数: %d\n', size(G.nodes, 1));
    fprintf('算法运行时间: %.2f 秒\n', elapsed_time);
end

% 保存最终的路径结果
save('rrt_result.mat', 'path', 'G', 'elapsed_time');
fprintf('结果已保存至 rrt_result.mat\n'); 