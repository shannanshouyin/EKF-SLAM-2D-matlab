clear; close all; clc;

%% 参数设置
num_landmarks = 100;        % 地标数量
max_range = 5;             % 传感器最大观测范围
sim_time = 500;             % 总仿真时间
dt = 0.5;                  % 时间步长
process_noise = diag([0.01, 0.0001]);        % 过程噪声
measure_noise = diag([0.15^2, (pi/180)^2]);  % 观测噪声
motion = [1; 0.1];         % 控制输入 [速度; 转向角速度]

%% 初始化
% 生成随机地标（均匀分布在场景中）
% rng(1); % 固定随机种子保证可重复性
true_landmarks_x = 30 * rand(num_landmarks,1) - 15; % 随机生成地标坐标
landmarks = [true_landmarks_x,20 * rand(num_landmarks,1)];

% 初始化状态
model_state = [0; 0; 0];     % 理想状态
true_state = [0; 0; 0];      % 真实状态 [x, y, theta]

est_state_S = [true_state; zeros(2 * num_landmarks,1)]; % 估计状态（包含地标）
covariance_S = eye(3 + 2*num_landmarks); % 协方差矩阵

est_state_B = [true_state; zeros(2 * num_landmarks,1)]; % 估计状态（包含地标）
covariance_B = eye(3 + 2*num_landmarks); % 协方差矩阵

% 存储历史轨迹
model_path = model_state(1:3);
true_path = true_state(1:3);
est_path_S = est_state_S(1:3);
est_path_B = est_state_B(1:3);

%% 主循环
for t = 0:dt:sim_time
    %% 真实系统运动（圆周运动）
    model_state = motion_model(model_state, motion, dt);
    true_state = motion_model_noise(true_state, motion, dt, process_noise);
    
    %% EKF预测步骤
    [est_state_S, covariance_S] = ekf_predict(est_state_S, covariance_S, motion, dt, process_noise);
    [est_state_B, covariance_B] = ekf_predict(est_state_B, covariance_B, motion, dt, process_noise);
    
    %% 观测处理
    visible_landmarks = find_visible_landmarks(true_state, landmarks, max_range);% 返回的是landmarks的索引
    measurements = get_measurements(true_state, landmarks, visible_landmarks, measure_noise);
    
    %% EKF更新步骤
    [est_state_S, covariance_S] = Iekf_update_S(est_state_S, covariance_S, measurements, measure_noise);
    [est_state_B, covariance_B] = Iekf_update_B(est_state_B, covariance_B, measurements, measure_noise);
    
    %% 记录轨迹
    model_path = [model_path, model_state(1:3)];
    true_path = [true_path, true_state(1:3)];
    est_path_S = [est_path_S, est_state_S(1:3)];
    est_path_B = [est_path_B, est_state_B(1:3)];
    
end

%% 计算均方根误差RMSE
for t = 1:sim_time 
    % 计算均方误差
    rmse_IEKF_S(t) = sqrt(mean((true_path(:,t) - est_path_S(:,t)).^2));
    rmse_IEKF_B(t) = sqrt(mean((true_path(:,t) - est_path_B(:,t)).^2));
    
    rmse_IEKF_S_pos(t) = sqrt(mean((true_path(1:2,t) - est_path_S(1:2,t)).^2));
    rmse_IEKF_B_pos(t) = sqrt(mean((true_path(1:2,t) - est_path_B(1:2,t)).^2));
    
    rmse_IEKF_S_theta(t) = sqrt(mean((true_path(3,t) - est_path_S(3,t)).^2));
    rmse_IEKF_B_theta(t) = sqrt(mean((true_path(3,t) - est_path_B(3,t)).^2));
end

figure; hold on;
plot(1:sim_time, rmse_IEKF_S, 'r-o','DisplayName', 'IEKF-S-Pose-RMES','LineWidth', 2);
plot(1:sim_time, rmse_IEKF_B, 'b-o','DisplayName', 'IEKF-B-Pose-RMSE','LineWidth', 2);
xlabel('time'); ylabel('RMSE(m)'); 
title('IEKF-S与IEKF-B位姿RMSE对比');
legend('Location', 'northeast');

figure; hold on;
subplot(2,1,1);
plot(1:sim_time, rmse_IEKF_S_pos, 'r-o','DisplayName', 'IEKF-S-Position-RMSE','LineWidth', 2);
hold on;
plot(1:sim_time, rmse_IEKF_B_pos, 'b-o','DisplayName', 'IEKF-B-Position-RMSE','LineWidth', 2);
xlabel('time'); ylabel('RMSE(m)'); 
title('IEKF-S与IEKF-B坐标RMSE对比');
legend('Location', 'northeast');

subplot(2,1,2);
plot(1:sim_time, rmse_IEKF_S_theta, 'r-o','DisplayName', 'IEKF-S-Heading-RMSE','LineWidth', 2);
hold on;
plot(1:sim_time, rmse_IEKF_B_theta, 'b-o','DisplayName', 'IEKF-B-Heading-RMSE','LineWidth', 2);
xlabel('time'); ylabel('RMSE(m)'); 
title('IEKF-S与IEKF-B朝向RMSE对比');
legend('Location', 'northeast');


IEKF_S_error = mean(rmse_IEKF_S);
IEKF_B_error = mean(rmse_IEKF_B);
disp("IEKF_S估计位姿RMSE误差为：");
disp(IEKF_S_error);
disp("IEKF_B估计位姿RMSE误差为：");
disp(IEKF_B_error);

IEKF_S_Position_error = mean(rmse_IEKF_S_pos);
IEKF_B_Position_error = mean(rmse_IEKF_B_pos);
disp("IEKF_S估计坐标RMSE误差为：");
disp(IEKF_S_Position_error);
disp("IEKF_B估计坐标RMSE误差为：");
disp(IEKF_B_Position_error);

IEKF_S_heading_error = mean(rmse_IEKF_S_theta);
IEKF_B_heading_error = mean(rmse_IEKF_B_theta);
disp("IEKF_S估计朝向RMSE误差为：");
disp(IEKF_S_heading_error);
disp("IEKF_B估计朝向RMSE误差为：");
disp(IEKF_B_heading_error);

