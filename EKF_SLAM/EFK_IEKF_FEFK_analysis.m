clear; close all; clc;

%% 参数设置
num_landmarks = 100;        % 地标数量
max_range = 5;             % 传感器最大观测范围
sim_time = 500;             % 总仿真时间
dt = 0.5;                  % 时间步长
process_noise = diag([0.01, 0.0001]);         % 过程噪声
measure_noise = diag([0.15^2, (pi/180)^2]);   % 观测噪声
motion = [1; 0.1];         % 控制输入 [速度; 转向角速度]

%% 初始化
% 生成随机地标（均匀分布在场景中）
% rng(1); % 固定随机种子保证可重复性
true_landmarks_x = 30 * rand(num_landmarks,1) - 15; % 随机生成地标坐标
landmarks = [true_landmarks_x,20 * rand(num_landmarks,1)];

% 初始化状态
model_state = [0; 0; 0];     % 理想状态
true_state = [0; 0; 0];      % 真实状态 [x, y, theta]
first_obs = [true_state; zeros(2 * num_landmarks,1)];
est_state_pre_F = [0;0;0];

est_state = [true_state; zeros(2 * num_landmarks,1)]; % 估计状态（包含地标）
covariance = eye(3 + 2*num_landmarks); % 协方差矩阵

est_state_I = [true_state; zeros(2 * num_landmarks,1)]; % 估计状态（包含地标）
covariance_I = eye(3 + 2*num_landmarks); % 协方差矩阵

est_state_F = [true_state; zeros(2 * num_landmarks,1)]; % 估计状态（包含地标）
covariance_F = eye(3 + 2*num_landmarks); % 协方差矩阵

% 存储历史轨迹
model_path = model_state(1:3);
true_path = true_state(1:3);
est_path = est_state(1:3);
est_path_I = est_state_I(1:3);
est_path_F = est_state_F(1:3);

%% 主循环
for t = 0:dt:sim_time
    %% 真实系统运动（圆周运动）
    model_state = motion_model(model_state, motion, dt);
    true_state = motion_model_noise(true_state, motion, dt, process_noise);
    
    %% EKF预测步骤
    [est_state, covariance] = ekf_predict(est_state, covariance, motion, dt, process_noise);
    [est_state_I, covariance_I] = ekf_predict(est_state_I, covariance_I, motion, dt, process_noise);
    [est_state_F, covariance_F] = fekf_predict(est_state_F, est_state_pre_F, covariance_F, motion, dt, process_noise);
    est_state_pre_F = est_state_F(1:3);
%     [est_state_F, covariance_F] = ekf_predict(est_state_F,covariance_F, motion, dt, process_noise);
    %% 观测处理
    visible_landmarks = find_visible_landmarks(true_state, landmarks, max_range);% 返回的是landmarks的索引
    measurements = get_measurements(true_state, landmarks, visible_landmarks, measure_noise);
    
    %% EKF更新步骤
    [est_state, covariance] = ekf_update(est_state, covariance, measurements, measure_noise);
    [est_state_I, covariance_I] = Iekf_update_B(est_state_I, covariance_I, measurements, measure_noise);
    [est_state_F, covariance_F,first_obs] = fekf_update(est_state_F, covariance_F, measurements, measure_noise,first_obs);
    %% 记录轨迹
    model_path = [model_path, model_state(1:3)];
    true_path = [true_path, true_state(1:3)];
    
    est_path = [est_path, est_state(1:3)];
    est_path_I = [est_path_I, est_state_I(1:3)];
    est_path_F = [est_path_F, est_state_F(1:3)];
   
end

%% 计算位姿误差RMSE
for t = 1:sim_time 
    % 计算均方误差
    rmse_true_EKF(t) = sqrt(mean((true_path(1:3,t) - est_path(1:3,t)).^2));
    rmse_true_EKF_pos(t) = sqrt(mean((true_path(1:2,t) - est_path(1:2,t)).^2));
    rmse_true_EKF_theta(t) = sqrt(mean((true_path(3,t) - est_path(3,t)).^2));
    
    rmse_true_EKF_I(t) = sqrt(mean((true_path(1:3,t) - est_path_I(1:3,t)).^2));
    rmse_true_EKF_I_pos(t) = sqrt(mean((true_path(1:2,t) - est_path_I(1:2,t)).^2));
    rmse_true_EKF_I_theta(t) = sqrt(mean((true_path(3,t) - est_path_I(3,t)).^2));
    
    rmse_true_EKF_F(t) = sqrt(mean((true_path(1:3,t) - est_path_F(1:3,t)).^2));
    rmse_true_EKF_F_pos(t) = sqrt(mean((true_path(1:2,t) - est_path_F(1:2,t)).^2));
    rmse_true_EKF_F_theta(t) = sqrt(mean((true_path(3,t) - est_path_F(3,t)).^2));
end

figure; hold on; grid on;
plot(1:sim_time, rmse_true_EKF, 'r-o','DisplayName', 'EKF-2norm-error','LineWidth', 2);
plot(1:sim_time, rmse_true_EKF_I, 'b-o','DisplayName', 'IEKF-2norm-error','LineWidth', 2);
plot(1:sim_time, rmse_true_EKF_F, 'm-o','DisplayName', 'FEJ-EKF-2norm-error','LineWidth', 2);
xlabel('time'); ylabel('RMSE(m)'); 
title('三种方法位姿RMSE对比');
legend('Location', 'northeast' );

figure; hold on; grid on; 
subplot(2,1,1);
plot(1:sim_time, rmse_true_EKF_pos, 'r-o','DisplayName', 'EKF-position-error','LineWidth', 2);
hold on;
plot(1:sim_time, rmse_true_EKF_I_pos, 'b-o','DisplayName', 'IEKF-position-error','LineWidth', 2);
hold on;
plot(1:sim_time, rmse_true_EKF_F_pos, 'm-o','DisplayName', 'FEJ-EKF-position-error','LineWidth', 2);
xlabel('time'); ylabel('RMSE(m)'); 
title('三种方法估计坐标RMSE对比');
legend('Location', 'northeast' );

subplot(2,1,2);
plot(1:sim_time, rmse_true_EKF_theta, 'r-o','DisplayName', 'EKF-heading-error','LineWidth', 2);
hold on;
plot(1:sim_time, rmse_true_EKF_I_theta, 'b-o','DisplayName', 'IEKF-heading-error','LineWidth', 2);
hold on;
plot(1:sim_time, rmse_true_EKF_F_theta, 'm-o','DisplayName', 'FEJ-EKF-heading-error','LineWidth', 2);
xlabel('time(s)'); ylabel('REMS(m)'); 
title('三种方法估计朝向误差对比');
legend('Location', 'northeast');

EKF_2norm_error = mean(rmse_true_EKF);
EKF_position_error = mean(rmse_true_EKF_pos);
EKF_heading_error = mean(rmse_true_EKF_theta);

IEKF_2norm_error = mean(rmse_true_EKF_I);
IEKF_position_error = mean(rmse_true_EKF_I_pos);
IEKF_heading_error = mean(rmse_true_EKF_I_theta);

FEKF_2norm_error = mean(rmse_true_EKF_F);
FEKF_position_error = mean(rmse_true_EKF_F_pos);
FEKF_heading_error = mean(rmse_true_EKF_F_theta);

disp("EKF估计平均二范数误差为：");
disp(EKF_2norm_error);
disp("IEKF估计平均二范数误差为：");
disp(IEKF_2norm_error);
disp("FEKF估计平均二范数误差为：");
disp(FEKF_2norm_error);

disp("EKF估计坐标平均RMSE为：");
disp(EKF_position_error);
disp("IEKF估计坐标平均RMSE为：");
disp(IEKF_position_error);
disp("FEKF估计坐标平均RMSE为：");
disp(FEKF_position_error);

disp("EKF估计朝向平均RMSE为：");
disp(EKF_heading_error);
disp("IEKF估计朝向平均RMSE为：");
disp(IEKF_heading_error);
disp("FEKF估计朝向平均RMSE为：");
disp(FEKF_heading_error);


%% 计算landmark直线误差

d_land = get_landmark_distance(est_state,landmarks);
d_land_I = get_landmark_distance(est_state_I,landmarks);
d_land_F = get_landmark_distance(est_state_F,landmarks);

EKF_derror_mean = mean(d_land);
IEKF_derror_mean = mean(d_land_I);
FEKF_derror_mean = mean(d_land_F);

disp("EKF建图平均RMSE为：");
disp(EKF_derror_mean);
disp("IEKF建图平均RMSE为：");
disp(IEKF_derror_mean);
disp("FEKF建图平均RMSE为：");
disp(FEKF_derror_mean);

figure; hold on;  
plot(d_land,'r-o','DisplayName','EFK-landmark-error','LineWidth', 2);
plot(d_land_I,'b-o','DisplayName','IEFK-landmark-error','LineWidth', 2);
plot(d_land_F,'m-o','DisplayName','FEJ-EFK-landmark-error','LineWidth', 2);
xlabel('landmarks'); ylabel('RMSE(m)'); 
title('三种方法地标RMSE对比');
legend('Location', 'northeast');