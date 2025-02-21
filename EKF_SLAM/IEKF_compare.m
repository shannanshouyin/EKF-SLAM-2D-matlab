clear; close all; clc;

%% ��������
num_landmarks = 100;        % �ر�����
max_range = 5;             % ���������۲ⷶΧ
sim_time = 500;             % �ܷ���ʱ��
dt = 0.5;                  % ʱ�䲽��
process_noise = diag([0.01, 0.0001]);        % ��������
measure_noise = diag([0.15^2, (pi/180)^2]);  % �۲�����
motion = [1; 0.1];         % �������� [�ٶ�; ת����ٶ�]

%% ��ʼ��
% ��������ر꣨���ȷֲ��ڳ����У�
% rng(1); % �̶�������ӱ�֤���ظ���
true_landmarks_x = 30 * rand(num_landmarks,1) - 15; % ������ɵر�����
landmarks = [true_landmarks_x,20 * rand(num_landmarks,1)];

% ��ʼ��״̬
model_state = [0; 0; 0];     % ����״̬
true_state = [0; 0; 0];      % ��ʵ״̬ [x, y, theta]

est_state_S = [true_state; zeros(2 * num_landmarks,1)]; % ����״̬�������ر꣩
covariance_S = eye(3 + 2*num_landmarks); % Э�������

est_state_B = [true_state; zeros(2 * num_landmarks,1)]; % ����״̬�������ر꣩
covariance_B = eye(3 + 2*num_landmarks); % Э�������

% �洢��ʷ�켣
model_path = model_state(1:3);
true_path = true_state(1:3);
est_path_S = est_state_S(1:3);
est_path_B = est_state_B(1:3);

%% ��ѭ��
for t = 0:dt:sim_time
    %% ��ʵϵͳ�˶���Բ���˶���
    model_state = motion_model(model_state, motion, dt);
    true_state = motion_model_noise(true_state, motion, dt, process_noise);
    
    %% EKFԤ�ⲽ��
    [est_state_S, covariance_S] = ekf_predict(est_state_S, covariance_S, motion, dt, process_noise);
    [est_state_B, covariance_B] = ekf_predict(est_state_B, covariance_B, motion, dt, process_noise);
    
    %% �۲⴦��
    visible_landmarks = find_visible_landmarks(true_state, landmarks, max_range);% ���ص���landmarks������
    measurements = get_measurements(true_state, landmarks, visible_landmarks, measure_noise);
    
    %% EKF���²���
    [est_state_S, covariance_S] = Iekf_update_S(est_state_S, covariance_S, measurements, measure_noise);
    [est_state_B, covariance_B] = Iekf_update_B(est_state_B, covariance_B, measurements, measure_noise);
    
    %% ��¼�켣
    model_path = [model_path, model_state(1:3)];
    true_path = [true_path, true_state(1:3)];
    est_path_S = [est_path_S, est_state_S(1:3)];
    est_path_B = [est_path_B, est_state_B(1:3)];
    
end

%% ������������RMSE
for t = 1:sim_time 
    % ����������
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
title('IEKF-S��IEKF-Bλ��RMSE�Ա�');
legend('Location', 'northeast');

figure; hold on;
subplot(2,1,1);
plot(1:sim_time, rmse_IEKF_S_pos, 'r-o','DisplayName', 'IEKF-S-Position-RMSE','LineWidth', 2);
hold on;
plot(1:sim_time, rmse_IEKF_B_pos, 'b-o','DisplayName', 'IEKF-B-Position-RMSE','LineWidth', 2);
xlabel('time'); ylabel('RMSE(m)'); 
title('IEKF-S��IEKF-B����RMSE�Ա�');
legend('Location', 'northeast');

subplot(2,1,2);
plot(1:sim_time, rmse_IEKF_S_theta, 'r-o','DisplayName', 'IEKF-S-Heading-RMSE','LineWidth', 2);
hold on;
plot(1:sim_time, rmse_IEKF_B_theta, 'b-o','DisplayName', 'IEKF-B-Heading-RMSE','LineWidth', 2);
xlabel('time'); ylabel('RMSE(m)'); 
title('IEKF-S��IEKF-B����RMSE�Ա�');
legend('Location', 'northeast');


IEKF_S_error = mean(rmse_IEKF_S);
IEKF_B_error = mean(rmse_IEKF_B);
disp("IEKF_S����λ��RMSE���Ϊ��");
disp(IEKF_S_error);
disp("IEKF_B����λ��RMSE���Ϊ��");
disp(IEKF_B_error);

IEKF_S_Position_error = mean(rmse_IEKF_S_pos);
IEKF_B_Position_error = mean(rmse_IEKF_B_pos);
disp("IEKF_S��������RMSE���Ϊ��");
disp(IEKF_S_Position_error);
disp("IEKF_B��������RMSE���Ϊ��");
disp(IEKF_B_Position_error);

IEKF_S_heading_error = mean(rmse_IEKF_S_theta);
IEKF_B_heading_error = mean(rmse_IEKF_B_theta);
disp("IEKF_S���Ƴ���RMSE���Ϊ��");
disp(IEKF_S_heading_error);
disp("IEKF_B���Ƴ���RMSE���Ϊ��");
disp(IEKF_B_heading_error);

