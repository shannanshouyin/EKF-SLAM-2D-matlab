clear; close all; clc;

%% 参数设置
% num_landmarks = 150;        % 地标数量
max_range = 5;             % 传感器最大观测范围
sim_time = 50;             % 总仿真时间
dt = 0.5;                  % 时间步长
process_noise = diag([0.01, 0.0001]);        % 过程噪声
measure_noise = diag([0.15^2, (pi/180)^2]);  % 观测噪声
motion = [1; 0.1];         % 控制输入 [速度; 转向角速度]


EKF_pose_error_statistics = [];
EKF_position_error_statistics = [];
EKF_heading_error_statistics = [];
EKF_distance_error_statistics = [];

n_min = 10;
dn = 2;
n_max = 200;
%% 初始化
for num_landmarks = n_min:dn:n_max
    % 生成随机地标（均匀分布在场景中）
    %rng(1); % 固定随机种子保证可重复性
    true_landmarks_x = 30 * rand(num_landmarks,1) - 15; % 随机生成地标坐标
    landmarks = [true_landmarks_x,20 * rand(num_landmarks,1)];

    % 初始化状态
    model_state = [0; 0; 0];     % 理想状态
    true_state = [0; 0; 0];      % 真实状态 [x, y, theta]

    est_state = [true_state; zeros(2 * num_landmarks,1)]; % 估计状态（包含地标）
    covariance = eye(3 + 2*num_landmarks); % 协方差矩阵

    % 存储历史轨迹
    model_path = model_state(1:3);
    true_path = true_state(1:3);
    est_path = est_state(1:3);

    %% 主循环
    n = 1;
    for t = 0:dt:sim_time
        %% 真实系统运动（圆周运动）
        model_state = motion_model(model_state, motion, dt);
        true_state = motion_model_noise(true_state, motion, dt, process_noise);

        %% EKF预测步骤
        [est_state, covariance] = ekf_predict(est_state, covariance, motion, dt, process_noise);

        %% 观测处理
        visible_landmarks = find_visible_landmarks(true_state, landmarks, max_range);% 返回的是landmarks的索引
        measurements = get_measurements(true_state, landmarks, visible_landmarks, measure_noise);
        % 这里获得的是按照带入真实路线获得的landmark实际测量值，下一步更新时带入估计路线以获得估计的landmark坐标

        %% EKF更新步骤
        [est_state, covariance] = ekf_update(est_state, covariance, measurements, measure_noise);

        %% 记录轨迹
        model_path = [model_path, model_state(1:3)];
        true_path = [true_path, true_state(1:3)];
        est_path = [est_path, est_state(1:3)];
        n = n+1;
        
        d_land = get_landmark_distance(est_state,landmarks);
    end

    %% 计算均方根误差RMSE
    for t = 1:sim_time 
        % 计算均方误差
        rmse_true_EKF(t) = sqrt(mean((true_path(:,t) - est_path(:,t)).^2));
        rmse_true_EKF_position(t) = sqrt(mean((true_path(1:2,t) - est_path(3,t)).^2));
        rmse_true_EKF_heading(t) = sqrt(mean((true_path(3,t) - est_path(3,t)).^2));
    end
    
    EKF_pose_error_mean = mean(rmse_true_EKF);
    EKF_position_error_mean = mean(rmse_true_EKF_position);
    EKF_heading_error_mean = mean(rmse_true_EKF_heading);
    EKF_derror_mean = mean(d_land);
    
%     disp("landmark数量为：");
%     disp(num_landmarks);
    
    EKF_pose_error_statistics = [EKF_pose_error_statistics,EKF_pose_error_mean];
    EKF_position_error_statistics = [EKF_position_error_statistics,EKF_position_error_mean];
    EKF_heading_error_statistics = [EKF_heading_error_statistics,EKF_heading_error_mean];
    EKF_distance_error_statistics = [EKF_distance_error_statistics,EKF_derror_mean];
end

figure; hold on;
% subplot(3,1,1);
plot(n_min:dn:n_max, EKF_pose_error_statistics, 'r-o','DisplayName', 'EFK-Pose-RMSE','LineWidth', 2);
xlabel('num-of-landmark'); ylabel('平均RMSE(m)'); 
title('landmark数量与EKF位姿平均RMSE关系');
legend('Location', 'northeast');
% subplot(3,1,2);
% plot(n_min:dn:n_max, EKF_position_error_statistics, 'r-o','DisplayName', 'EFK-Position-RMSE','LineWidth', 2);
% xlabel('num-of-landmark'); ylabel('平均RMSE(m)'); 
% title('landmark数量与EKF坐标平均RMSE关系');
% legend('Location', 'northeast');
% subplot(3,1,3);
% plot(n_min:dn:n_max, EKF_heading_error_statistics, 'r-o','DisplayName', 'EFK-Heading-RMSE','LineWidth', 2);
% xlabel('num-of-landmark'); ylabel('平均RMSE(m)'); 
% title('landmark数量与EKF朝向平均RMSE关系');
% legend('Location', 'northeast');

figure; hold on;
plot(EKF_distance_error_statistics, 'r-o','DisplayName', 'EFK-Landmark-RMSE','LineWidth', 2);
xlabel('num-of-landmark'); ylabel('平均RMSE(m)'); 
title('landmark数量与地标平均RMSE关系');
legend('Location', 'northeast');
