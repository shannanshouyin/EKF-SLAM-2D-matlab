clear; close all; clc;

%% 参数设置
num_landmarks = 100;        % 地标数量
max_range = 5;             % 传感器最大观测范围
sim_time = 50;             % 总仿真时间
dt = 0.5;                  % 时间步长
process_noise = diag([0.01, 0.0001]);       	 % 过程噪声
measure_noise = diag([0.15^2, (pi/180)^2]);  % 观测噪声
motion = [1; 0.1];         % 控制输入 [速度; 转向角速度]

%% 初始化
% 生成随机地标（均匀分布在场景中）
%rng(1); % 固定随机种子保证可重复性
true_landmarks_x = 30 * rand(num_landmarks,1) - 15; % 随机生成地标坐标
landmarks = [true_landmarks_x,20 * rand(num_landmarks,1)];

% landmarks = [5, 5]; % 地标位置 [x1, y1; x2, y2; ...]
% num_landmarks = size(landmarks, 1);

% 初始化状态
model_state = [0; 0; 0];     % 理想状态
true_state = [0; 0; 0];      % 真实状态 [x, y, theta]
est_state = [true_state; zeros(2 * num_landmarks,1)]; % 估计状态（包含地标）
first_obs = zeros(2*num_landmarks+3,2);
est_state_pre = [0;0;0];
covariance = eye(3 + 2*num_landmarks); % 协方差矩阵

% 存储历史轨迹
model_path = model_state(1:3);
true_path = true_state(1:3);
est_path = est_state(1:3);

% 图形初始化
figure; hold on; grid on; axis equal;
% 绘制路径
h_model = plot(model_state(1), model_state(2), 'g.-', 'DisplayName', '模型路径','LineWidth', 2);
h_actual = plot(true_state(1), true_state(2), 'b.-', 'DisplayName', '实际路径','LineWidth', 2);
h_est = plot(est_state(1), est_state(2), 'r.-', 'DisplayName', '估计路径','LineWidth', 2);

% 绘制朝向箭头
arrow_length = 2;
h_model_orientation = quiver(model_state(1), model_state(2),arrow_length*cos(model_state(3)),...
                           arrow_length*sin(model_state(3)),'g', 'LineWidth', 2, 'MaxHeadSize', 1, 'DisplayName', '模型朝向');
h_true_orientation = quiver(true_state(1), true_state(2),arrow_length*cos(true_state(3)),...
                           arrow_length*sin(true_state(3)),'b', 'LineWidth', 2, 'MaxHeadSize', 1, 'DisplayName', '真实朝向');
h_est_orientation = quiver(est_state(1), est_state(2),arrow_length*cos(est_state(3)),...
                          arrow_length*sin(est_state(3)), 'r', 'LineWidth', 2, 'MaxHeadSize', 1, 'DisplayName', '估计朝向');

% 绘制基础图形                      
h_car = plot(true_state(1), true_state(2), 'ro', 'MarkerSize',8, 'LineWidth',2);
h_range = rectangle('Position',[true_state(1)-max_range, true_state(2)-max_range,...
                 2*max_range, 2*max_range], 'Curvature',[1,1], 'EdgeColor','b');
h_landmarks = plot(landmarks(:,1), landmarks(:,2), 'k^', 'MarkerSize',8,...
                 'DisplayName', '真实地标');
             
h_obs_lines = gobjects(0); % 存储观测连线的图形句柄
h_est_landmarks = gobjects(0); % 估计地标的图形句柄
m_line = gobjects(0); % 估计地标的图形句柄
legend('Location', 'best'); 
xlabel('X'); ylabel('Y'); 
title('EKF-SLAM 仿真');

%% 主循环
for t = 0:dt:sim_time
    %% 真实系统运动（圆周运动）
    model_state = motion_model(model_state, motion, dt);
    true_state = motion_model_noise(true_state, motion, dt, process_noise);
    
    %% EKF预测步骤
    [est_state, covariance] = fekf_predict(est_state, est_state_pre, covariance, motion, dt, process_noise);
    est_state_pre = est_state(1:3);
    %% 观测处理
    visible_landmarks = find_visible_landmarks(true_state, landmarks, max_range);% 返回的是landmarks的索引
    measurements = get_measurements(true_state, landmarks, visible_landmarks, measure_noise);
 
    %% EKF更新步骤
    [est_state, P, first_obs] = fekf_update(est_state, covariance ,measurements, measure_noise, first_obs);

    %% 记录轨迹
    model_path = [model_path, model_state(1:3)];
    true_path = [true_path, true_state(1:3)];
    est_path = [est_path, est_state(1:3)];
    %% 动态可视化
    % 更新小车位置和观测范围
    set(h_car, 'XData', true_state(1), 'YData', true_state(2));
    set(h_range, 'Position', [true_state(1)-max_range, true_state(2)-max_range,...
                              2*max_range, 2*max_range]);
    
    % 更新路径
    set(h_model, 'XData', model_path(1,:), 'YData', model_path(2,:));
    set(h_actual, 'XData', true_path(1,:), 'YData', true_path(2,:));
    set(h_est, 'XData', est_path(1,:), 'YData', est_path(2,:));
    
    % 更新朝向箭头
    set(h_model_orientation, 'XData', model_state(1), 'YData', model_state(2),...
        'UData', arrow_length*cos(model_state(3)), 'VData', arrow_length*sin(model_state(3)));
    set(h_true_orientation, 'XData', true_state(1), 'YData', true_state(2),...
        'UData', arrow_length*cos(true_state(3)), 'VData', arrow_length*sin(true_state(3)));
    set(h_est_orientation,'XData', est_state(1), 'YData', est_state(2),...
        'UData', arrow_length*cos(est_state(3)),'VData', arrow_length*sin(est_state(3)));
    
    % 绘制估计的地标位置
    delete(h_est_landmarks);
    delete(m_line);
    [h_est_landmarks,m_line] = plot_estimated_landmarks(est_state,landmarks);
    
    % 绘制观测连线
    h_obs_lines = update_observation_lines(true_state, landmarks, visible_landmarks, h_obs_lines);
    
    drawnow;
    pause(0.001);
end

%% 计算均方根误差RMSE
for t = 1:sim_time 
    % 计算均方误差
    rmse_true_EKF(t) = sqrt(mean((true_path(:,t) - est_path(:,t)).^2));
    rmse_true_model(t) = sqrt(mean((true_path(:,t) - model_path(:,t)).^2));
end

figure; hold on; grid on; 
plot(1:sim_time, rmse_true_EKF, 'r-o','DisplayName', 'EKF-error','LineWidth', 2);
plot(1:sim_time, rmse_true_model, 'b-o','DisplayName', 'model-error','LineWidth', 2);
xlabel('epoch'); ylabel('REMS(m)'); 
title('真实路径与估计路径误差');
legend('Location', 'best');

EKF_error_mean = mean(rmse_true_EKF);
disp("landmark数量为：");
disp(num_landmarks);
disp("EKF估计路径平均RMSE误差为：");
disp(EKF_error_mean);