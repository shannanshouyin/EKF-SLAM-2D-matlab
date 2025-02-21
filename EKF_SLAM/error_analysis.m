clear; close all; clc;

%% 参数设置
% num_landmarks = 100;        % 地标数量
max_range = 5;             % 传感器最大观测范围
sim_time = 100;             % 总仿真时间
dt = 0.5;                  % 时间步长
process_noise = diag([0.01, 0.01]);          % 过程噪声
measure_noise = diag([0.15^2, (pi/180)^2]);  % 观测噪声
motion = [1; 0.1];         % 控制输入 [速度; 转向角速度]

EKF_2norm_error_statistics = [];
EKF_x_error_statistics = [];
EKF_y_error_statistics = [];
EKF_angle_error_statistics = [];

mod_2norm_error_statistics = [];
mod_x_error_statistics = [];
mod_y_error_statistics = [];
mod_angle_error_statistics = [];

n_min = 10;
dn = 10;
n_max = 100;
%% 初始化
for num_landmarks = n_min:dn:n_max
    % 生成随机地标（均匀分布在场景中）
    % rng(1); % 固定随机种子保证可重复性
    true_landmarks_x = 30 * rand(num_landmarks,1) - 15; % 随机生成地标坐标
    landmarks = [true_landmarks_x,20 * rand(num_landmarks,1)];

    % landmarks = [5, 5; -5, 5;7, 7;-7, 7;10, -10;10,10]; % 地标位置 [x1, y1; x2, y2; ...]
    % num_landmarks = size(landmarks, 1);

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
    end

    %% 计算均方根误差RMSE
    for t = 1:sim_time 
        % 计算均方误差
        EKF_2norm_error(t) = sqrt(mean((true_path(:,t) - est_path(:,t)).^2));
        EKF_x_error(t) = abs(true_path(1,t) - est_path(1,t));
        EKF_y_error(t) = abs(true_path(2,t) - est_path(2,t));
        EKF_angle_error(t) = abs(true_path(3,t) - est_path(3,t));
        
        mod_2norm_error(t) = sqrt(mean((true_path(:,t) - model_path(:,t)).^2));
        mod_x_error(t) = abs(true_path(1,t) - model_path(1,t));
        mod_y_error(t) = abs(true_path(2,t) - model_path(2,t));
        mod_angle_error(t) = abs(true_path(3,t) - model_path(3,t));
    end

    EKF_2norm_error_mean = mean(EKF_2norm_error);
    EKF_x_error_mean = mean(EKF_x_error);
    EKF_y_error_mean = mean(EKF_y_error);
    EKF_angle_error_mean = mean(EKF_angle_error);
    
    mod_2norm_error_mean = mean(mod_2norm_error);
    mod_x_error_mean = mean(mod_x_error);
    mod_y_error_mean = mean(mod_y_error);
    mod_angle_error_mean = mean(mod_angle_error);
    
    disp("landmark数量为：");
    disp(num_landmarks);
    disp("EKF估计路径平均RMSE误差为：");
    disp(EKF_2norm_error_mean);
    
    EKF_2norm_error_statistics = [EKF_2norm_error_statistics,EKF_2norm_error_mean];
    EKF_x_error_statistics = [EKF_x_error_statistics,EKF_x_error_mean];
    EKF_y_error_statistics = [EKF_y_error_statistics,EKF_y_error_mean];
    EKF_angle_error_statistics = [EKF_angle_error_statistics,EKF_angle_error_mean];
    
    mod_2norm_error_statistics = [mod_2norm_error_statistics,mod_2norm_error_mean];
    mod_x_error_statistics = [mod_x_error_statistics,mod_x_error_mean];
    mod_y_error_statistics = [mod_y_error_statistics,mod_y_error_mean];
    mod_angle_error_statistics = [mod_angle_error_statistics,mod_angle_error_mean];
end

figure; hold on; grid on; 
plot(n_min:dn:n_max, EKF_2norm_error_statistics, 'r-o','DisplayName', 'EKF-x-error','LineWidth', 2);
hold on;
plot(n_min:dn:n_max, mod_2norm_error_statistics, 'b-o','DisplayName', 'MODEL-x-error','LineWidth', 2);
xlabel('num_landmark'); ylabel('平均二范数误差(m)'); 
title('landmark数量与EKF二范数误差关系');
legend('Location', 'best');

figure; hold on; grid on;
subplot(3, 1, 1);
plot(n_min:dn:n_max, EKF_x_error_statistics, 'r-o','DisplayName', 'EKF-x-error','LineWidth', 2);
hold on;
plot(n_min:dn:n_max, mod_x_error_statistics, 'b-o','DisplayName', 'MODEL-x-error','LineWidth', 2);
ylabel('x平均误差(m)');
legend('Location', 'best');
subplot(3, 1, 2);
plot(n_min:dn:n_max, EKF_y_error_statistics, 'r-o','DisplayName', 'EKF-y-error','LineWidth', 2);
hold on;
plot(n_min:dn:n_max, mod_y_error_statistics, 'b-o','DisplayName', 'MODEL-y-error','LineWidth', 2);
ylabel('y平均误差(m)');
legend('Location', 'best');
subplot(3, 1, 3);
plot(n_min:dn:n_max, EKF_angle_error_statistics, 'r-o','DisplayName', 'EKF-angle-error','LineWidth', 2);
hold on;
plot(n_min:dn:n_max, mod_angle_error_statistics, 'b-o','DisplayName', 'MODEL-angle-error','LineWidth', 2);
ylabel('朝向角平均平均误差(m)');
xlabel('num_landmark');
title('landmark数量与EKF各项误差关系');
legend('Location', 'best');