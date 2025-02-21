clear; close all; clc;

%% ��������
% num_landmarks = 150;        % �ر�����
max_range = 5;             % ���������۲ⷶΧ
sim_time = 50;             % �ܷ���ʱ��
dt = 0.5;                  % ʱ�䲽��
process_noise = diag([0.01, 0.0001]);        % ��������
measure_noise = diag([0.15^2, (pi/180)^2]);  % �۲�����
motion = [1; 0.1];         % �������� [�ٶ�; ת����ٶ�]


EKF_pose_error_statistics = [];
EKF_position_error_statistics = [];
EKF_heading_error_statistics = [];
EKF_distance_error_statistics = [];

n_min = 10;
dn = 2;
n_max = 200;
%% ��ʼ��
for num_landmarks = n_min:dn:n_max
    % ��������ر꣨���ȷֲ��ڳ����У�
    %rng(1); % �̶�������ӱ�֤���ظ���
    true_landmarks_x = 30 * rand(num_landmarks,1) - 15; % ������ɵر�����
    landmarks = [true_landmarks_x,20 * rand(num_landmarks,1)];

    % ��ʼ��״̬
    model_state = [0; 0; 0];     % ����״̬
    true_state = [0; 0; 0];      % ��ʵ״̬ [x, y, theta]

    est_state = [true_state; zeros(2 * num_landmarks,1)]; % ����״̬�������ر꣩
    covariance = eye(3 + 2*num_landmarks); % Э�������

    % �洢��ʷ�켣
    model_path = model_state(1:3);
    true_path = true_state(1:3);
    est_path = est_state(1:3);

    %% ��ѭ��
    n = 1;
    for t = 0:dt:sim_time
        %% ��ʵϵͳ�˶���Բ���˶���
        model_state = motion_model(model_state, motion, dt);
        true_state = motion_model_noise(true_state, motion, dt, process_noise);

        %% EKFԤ�ⲽ��
        [est_state, covariance] = ekf_predict(est_state, covariance, motion, dt, process_noise);

        %% �۲⴦��
        visible_landmarks = find_visible_landmarks(true_state, landmarks, max_range);% ���ص���landmarks������
        measurements = get_measurements(true_state, landmarks, visible_landmarks, measure_noise);
        % �����õ��ǰ��մ�����ʵ·�߻�õ�landmarkʵ�ʲ���ֵ����һ������ʱ�������·���Ի�ù��Ƶ�landmark����

        %% EKF���²���
        [est_state, covariance] = ekf_update(est_state, covariance, measurements, measure_noise);

        %% ��¼�켣
        model_path = [model_path, model_state(1:3)];
        true_path = [true_path, true_state(1:3)];
        est_path = [est_path, est_state(1:3)];
        n = n+1;
        
        d_land = get_landmark_distance(est_state,landmarks);
    end

    %% ������������RMSE
    for t = 1:sim_time 
        % ����������
        rmse_true_EKF(t) = sqrt(mean((true_path(:,t) - est_path(:,t)).^2));
        rmse_true_EKF_position(t) = sqrt(mean((true_path(1:2,t) - est_path(3,t)).^2));
        rmse_true_EKF_heading(t) = sqrt(mean((true_path(3,t) - est_path(3,t)).^2));
    end
    
    EKF_pose_error_mean = mean(rmse_true_EKF);
    EKF_position_error_mean = mean(rmse_true_EKF_position);
    EKF_heading_error_mean = mean(rmse_true_EKF_heading);
    EKF_derror_mean = mean(d_land);
    
%     disp("landmark����Ϊ��");
%     disp(num_landmarks);
    
    EKF_pose_error_statistics = [EKF_pose_error_statistics,EKF_pose_error_mean];
    EKF_position_error_statistics = [EKF_position_error_statistics,EKF_position_error_mean];
    EKF_heading_error_statistics = [EKF_heading_error_statistics,EKF_heading_error_mean];
    EKF_distance_error_statistics = [EKF_distance_error_statistics,EKF_derror_mean];
end

figure; hold on;
% subplot(3,1,1);
plot(n_min:dn:n_max, EKF_pose_error_statistics, 'r-o','DisplayName', 'EFK-Pose-RMSE','LineWidth', 2);
xlabel('num-of-landmark'); ylabel('ƽ��RMSE(m)'); 
title('landmark������EKFλ��ƽ��RMSE��ϵ');
legend('Location', 'northeast');
% subplot(3,1,2);
% plot(n_min:dn:n_max, EKF_position_error_statistics, 'r-o','DisplayName', 'EFK-Position-RMSE','LineWidth', 2);
% xlabel('num-of-landmark'); ylabel('ƽ��RMSE(m)'); 
% title('landmark������EKF����ƽ��RMSE��ϵ');
% legend('Location', 'northeast');
% subplot(3,1,3);
% plot(n_min:dn:n_max, EKF_heading_error_statistics, 'r-o','DisplayName', 'EFK-Heading-RMSE','LineWidth', 2);
% xlabel('num-of-landmark'); ylabel('ƽ��RMSE(m)'); 
% title('landmark������EKF����ƽ��RMSE��ϵ');
% legend('Location', 'northeast');

figure; hold on;
plot(EKF_distance_error_statistics, 'r-o','DisplayName', 'EFK-Landmark-RMSE','LineWidth', 2);
xlabel('num-of-landmark'); ylabel('ƽ��RMSE(m)'); 
title('landmark������ر�ƽ��RMSE��ϵ');
legend('Location', 'northeast');
