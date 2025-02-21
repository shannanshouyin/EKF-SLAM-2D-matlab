clear; close all; clc;

%% ��������
% num_landmarks = 100;        % �ر�����
max_range = 5;             % ���������۲ⷶΧ
sim_time = 100;             % �ܷ���ʱ��
dt = 0.5;                  % ʱ�䲽��
process_noise = diag([0.01, 0.01]);          % ��������
measure_noise = diag([0.15^2, (pi/180)^2]);  % �۲�����
motion = [1; 0.1];         % �������� [�ٶ�; ת����ٶ�]

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
%% ��ʼ��
for num_landmarks = n_min:dn:n_max
    % ��������ر꣨���ȷֲ��ڳ����У�
    % rng(1); % �̶�������ӱ�֤���ظ���
    true_landmarks_x = 30 * rand(num_landmarks,1) - 15; % ������ɵر�����
    landmarks = [true_landmarks_x,20 * rand(num_landmarks,1)];

    % landmarks = [5, 5; -5, 5;7, 7;-7, 7;10, -10;10,10]; % �ر�λ�� [x1, y1; x2, y2; ...]
    % num_landmarks = size(landmarks, 1);

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
    end

    %% ������������RMSE
    for t = 1:sim_time 
        % ����������
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
    
    disp("landmark����Ϊ��");
    disp(num_landmarks);
    disp("EKF����·��ƽ��RMSE���Ϊ��");
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
xlabel('num_landmark'); ylabel('ƽ�����������(m)'); 
title('landmark������EKF����������ϵ');
legend('Location', 'best');

figure; hold on; grid on;
subplot(3, 1, 1);
plot(n_min:dn:n_max, EKF_x_error_statistics, 'r-o','DisplayName', 'EKF-x-error','LineWidth', 2);
hold on;
plot(n_min:dn:n_max, mod_x_error_statistics, 'b-o','DisplayName', 'MODEL-x-error','LineWidth', 2);
ylabel('xƽ�����(m)');
legend('Location', 'best');
subplot(3, 1, 2);
plot(n_min:dn:n_max, EKF_y_error_statistics, 'r-o','DisplayName', 'EKF-y-error','LineWidth', 2);
hold on;
plot(n_min:dn:n_max, mod_y_error_statistics, 'b-o','DisplayName', 'MODEL-y-error','LineWidth', 2);
ylabel('yƽ�����(m)');
legend('Location', 'best');
subplot(3, 1, 3);
plot(n_min:dn:n_max, EKF_angle_error_statistics, 'r-o','DisplayName', 'EKF-angle-error','LineWidth', 2);
hold on;
plot(n_min:dn:n_max, mod_angle_error_statistics, 'b-o','DisplayName', 'MODEL-angle-error','LineWidth', 2);
ylabel('�����ƽ��ƽ�����(m)');
xlabel('num_landmark');
title('landmark������EKF��������ϵ');
legend('Location', 'best');