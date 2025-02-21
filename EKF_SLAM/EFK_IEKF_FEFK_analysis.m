clear; close all; clc;

%% ��������
num_landmarks = 100;        % �ر�����
max_range = 5;             % ���������۲ⷶΧ
sim_time = 500;             % �ܷ���ʱ��
dt = 0.5;                  % ʱ�䲽��
process_noise = diag([0.01, 0.0001]);         % ��������
measure_noise = diag([0.15^2, (pi/180)^2]);   % �۲�����
motion = [1; 0.1];         % �������� [�ٶ�; ת����ٶ�]

%% ��ʼ��
% ��������ر꣨���ȷֲ��ڳ����У�
% rng(1); % �̶�������ӱ�֤���ظ���
true_landmarks_x = 30 * rand(num_landmarks,1) - 15; % ������ɵر�����
landmarks = [true_landmarks_x,20 * rand(num_landmarks,1)];

% ��ʼ��״̬
model_state = [0; 0; 0];     % ����״̬
true_state = [0; 0; 0];      % ��ʵ״̬ [x, y, theta]
first_obs = [true_state; zeros(2 * num_landmarks,1)];
est_state_pre_F = [0;0;0];

est_state = [true_state; zeros(2 * num_landmarks,1)]; % ����״̬�������ر꣩
covariance = eye(3 + 2*num_landmarks); % Э�������

est_state_I = [true_state; zeros(2 * num_landmarks,1)]; % ����״̬�������ر꣩
covariance_I = eye(3 + 2*num_landmarks); % Э�������

est_state_F = [true_state; zeros(2 * num_landmarks,1)]; % ����״̬�������ر꣩
covariance_F = eye(3 + 2*num_landmarks); % Э�������

% �洢��ʷ�켣
model_path = model_state(1:3);
true_path = true_state(1:3);
est_path = est_state(1:3);
est_path_I = est_state_I(1:3);
est_path_F = est_state_F(1:3);

%% ��ѭ��
for t = 0:dt:sim_time
    %% ��ʵϵͳ�˶���Բ���˶���
    model_state = motion_model(model_state, motion, dt);
    true_state = motion_model_noise(true_state, motion, dt, process_noise);
    
    %% EKFԤ�ⲽ��
    [est_state, covariance] = ekf_predict(est_state, covariance, motion, dt, process_noise);
    [est_state_I, covariance_I] = ekf_predict(est_state_I, covariance_I, motion, dt, process_noise);
    [est_state_F, covariance_F] = fekf_predict(est_state_F, est_state_pre_F, covariance_F, motion, dt, process_noise);
    est_state_pre_F = est_state_F(1:3);
%     [est_state_F, covariance_F] = ekf_predict(est_state_F,covariance_F, motion, dt, process_noise);
    %% �۲⴦��
    visible_landmarks = find_visible_landmarks(true_state, landmarks, max_range);% ���ص���landmarks������
    measurements = get_measurements(true_state, landmarks, visible_landmarks, measure_noise);
    
    %% EKF���²���
    [est_state, covariance] = ekf_update(est_state, covariance, measurements, measure_noise);
    [est_state_I, covariance_I] = Iekf_update_B(est_state_I, covariance_I, measurements, measure_noise);
    [est_state_F, covariance_F,first_obs] = fekf_update(est_state_F, covariance_F, measurements, measure_noise,first_obs);
    %% ��¼�켣
    model_path = [model_path, model_state(1:3)];
    true_path = [true_path, true_state(1:3)];
    
    est_path = [est_path, est_state(1:3)];
    est_path_I = [est_path_I, est_state_I(1:3)];
    est_path_F = [est_path_F, est_state_F(1:3)];
   
end

%% ����λ�����RMSE
for t = 1:sim_time 
    % ����������
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
title('���ַ���λ��RMSE�Ա�');
legend('Location', 'northeast' );

figure; hold on; grid on; 
subplot(2,1,1);
plot(1:sim_time, rmse_true_EKF_pos, 'r-o','DisplayName', 'EKF-position-error','LineWidth', 2);
hold on;
plot(1:sim_time, rmse_true_EKF_I_pos, 'b-o','DisplayName', 'IEKF-position-error','LineWidth', 2);
hold on;
plot(1:sim_time, rmse_true_EKF_F_pos, 'm-o','DisplayName', 'FEJ-EKF-position-error','LineWidth', 2);
xlabel('time'); ylabel('RMSE(m)'); 
title('���ַ�����������RMSE�Ա�');
legend('Location', 'northeast' );

subplot(2,1,2);
plot(1:sim_time, rmse_true_EKF_theta, 'r-o','DisplayName', 'EKF-heading-error','LineWidth', 2);
hold on;
plot(1:sim_time, rmse_true_EKF_I_theta, 'b-o','DisplayName', 'IEKF-heading-error','LineWidth', 2);
hold on;
plot(1:sim_time, rmse_true_EKF_F_theta, 'm-o','DisplayName', 'FEJ-EKF-heading-error','LineWidth', 2);
xlabel('time(s)'); ylabel('REMS(m)'); 
title('���ַ������Ƴ������Ա�');
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

disp("EKF����ƽ�����������Ϊ��");
disp(EKF_2norm_error);
disp("IEKF����ƽ�����������Ϊ��");
disp(IEKF_2norm_error);
disp("FEKF����ƽ�����������Ϊ��");
disp(FEKF_2norm_error);

disp("EKF��������ƽ��RMSEΪ��");
disp(EKF_position_error);
disp("IEKF��������ƽ��RMSEΪ��");
disp(IEKF_position_error);
disp("FEKF��������ƽ��RMSEΪ��");
disp(FEKF_position_error);

disp("EKF���Ƴ���ƽ��RMSEΪ��");
disp(EKF_heading_error);
disp("IEKF���Ƴ���ƽ��RMSEΪ��");
disp(IEKF_heading_error);
disp("FEKF���Ƴ���ƽ��RMSEΪ��");
disp(FEKF_heading_error);


%% ����landmarkֱ�����

d_land = get_landmark_distance(est_state,landmarks);
d_land_I = get_landmark_distance(est_state_I,landmarks);
d_land_F = get_landmark_distance(est_state_F,landmarks);

EKF_derror_mean = mean(d_land);
IEKF_derror_mean = mean(d_land_I);
FEKF_derror_mean = mean(d_land_F);

disp("EKF��ͼƽ��RMSEΪ��");
disp(EKF_derror_mean);
disp("IEKF��ͼƽ��RMSEΪ��");
disp(IEKF_derror_mean);
disp("FEKF��ͼƽ��RMSEΪ��");
disp(FEKF_derror_mean);

figure; hold on;  
plot(d_land,'r-o','DisplayName','EFK-landmark-error','LineWidth', 2);
plot(d_land_I,'b-o','DisplayName','IEFK-landmark-error','LineWidth', 2);
plot(d_land_F,'m-o','DisplayName','FEJ-EFK-landmark-error','LineWidth', 2);
xlabel('landmarks'); ylabel('RMSE(m)'); 
title('���ַ����ر�RMSE�Ա�');
legend('Location', 'northeast');