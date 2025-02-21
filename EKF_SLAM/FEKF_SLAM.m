clear; close all; clc;

%% ��������
num_landmarks = 100;        % �ر�����
max_range = 5;             % ���������۲ⷶΧ
sim_time = 50;             % �ܷ���ʱ��
dt = 0.5;                  % ʱ�䲽��
process_noise = diag([0.01, 0.0001]);       	 % ��������
measure_noise = diag([0.15^2, (pi/180)^2]);  % �۲�����
motion = [1; 0.1];         % �������� [�ٶ�; ת����ٶ�]

%% ��ʼ��
% ��������ر꣨���ȷֲ��ڳ����У�
%rng(1); % �̶�������ӱ�֤���ظ���
true_landmarks_x = 30 * rand(num_landmarks,1) - 15; % ������ɵر�����
landmarks = [true_landmarks_x,20 * rand(num_landmarks,1)];

% landmarks = [5, 5]; % �ر�λ�� [x1, y1; x2, y2; ...]
% num_landmarks = size(landmarks, 1);

% ��ʼ��״̬
model_state = [0; 0; 0];     % ����״̬
true_state = [0; 0; 0];      % ��ʵ״̬ [x, y, theta]
est_state = [true_state; zeros(2 * num_landmarks,1)]; % ����״̬�������ر꣩
first_obs = zeros(2*num_landmarks+3,2);
est_state_pre = [0;0;0];
covariance = eye(3 + 2*num_landmarks); % Э�������

% �洢��ʷ�켣
model_path = model_state(1:3);
true_path = true_state(1:3);
est_path = est_state(1:3);

% ͼ�γ�ʼ��
figure; hold on; grid on; axis equal;
% ����·��
h_model = plot(model_state(1), model_state(2), 'g.-', 'DisplayName', 'ģ��·��','LineWidth', 2);
h_actual = plot(true_state(1), true_state(2), 'b.-', 'DisplayName', 'ʵ��·��','LineWidth', 2);
h_est = plot(est_state(1), est_state(2), 'r.-', 'DisplayName', '����·��','LineWidth', 2);

% ���Ƴ����ͷ
arrow_length = 2;
h_model_orientation = quiver(model_state(1), model_state(2),arrow_length*cos(model_state(3)),...
                           arrow_length*sin(model_state(3)),'g', 'LineWidth', 2, 'MaxHeadSize', 1, 'DisplayName', 'ģ�ͳ���');
h_true_orientation = quiver(true_state(1), true_state(2),arrow_length*cos(true_state(3)),...
                           arrow_length*sin(true_state(3)),'b', 'LineWidth', 2, 'MaxHeadSize', 1, 'DisplayName', '��ʵ����');
h_est_orientation = quiver(est_state(1), est_state(2),arrow_length*cos(est_state(3)),...
                          arrow_length*sin(est_state(3)), 'r', 'LineWidth', 2, 'MaxHeadSize', 1, 'DisplayName', '���Ƴ���');

% ���ƻ���ͼ��                      
h_car = plot(true_state(1), true_state(2), 'ro', 'MarkerSize',8, 'LineWidth',2);
h_range = rectangle('Position',[true_state(1)-max_range, true_state(2)-max_range,...
                 2*max_range, 2*max_range], 'Curvature',[1,1], 'EdgeColor','b');
h_landmarks = plot(landmarks(:,1), landmarks(:,2), 'k^', 'MarkerSize',8,...
                 'DisplayName', '��ʵ�ر�');
             
h_obs_lines = gobjects(0); % �洢�۲����ߵ�ͼ�ξ��
h_est_landmarks = gobjects(0); % ���Ƶر��ͼ�ξ��
m_line = gobjects(0); % ���Ƶر��ͼ�ξ��
legend('Location', 'best'); 
xlabel('X'); ylabel('Y'); 
title('EKF-SLAM ����');

%% ��ѭ��
for t = 0:dt:sim_time
    %% ��ʵϵͳ�˶���Բ���˶���
    model_state = motion_model(model_state, motion, dt);
    true_state = motion_model_noise(true_state, motion, dt, process_noise);
    
    %% EKFԤ�ⲽ��
    [est_state, covariance] = fekf_predict(est_state, est_state_pre, covariance, motion, dt, process_noise);
    est_state_pre = est_state(1:3);
    %% �۲⴦��
    visible_landmarks = find_visible_landmarks(true_state, landmarks, max_range);% ���ص���landmarks������
    measurements = get_measurements(true_state, landmarks, visible_landmarks, measure_noise);
 
    %% EKF���²���
    [est_state, P, first_obs] = fekf_update(est_state, covariance ,measurements, measure_noise, first_obs);

    %% ��¼�켣
    model_path = [model_path, model_state(1:3)];
    true_path = [true_path, true_state(1:3)];
    est_path = [est_path, est_state(1:3)];
    %% ��̬���ӻ�
    % ����С��λ�ú͹۲ⷶΧ
    set(h_car, 'XData', true_state(1), 'YData', true_state(2));
    set(h_range, 'Position', [true_state(1)-max_range, true_state(2)-max_range,...
                              2*max_range, 2*max_range]);
    
    % ����·��
    set(h_model, 'XData', model_path(1,:), 'YData', model_path(2,:));
    set(h_actual, 'XData', true_path(1,:), 'YData', true_path(2,:));
    set(h_est, 'XData', est_path(1,:), 'YData', est_path(2,:));
    
    % ���³����ͷ
    set(h_model_orientation, 'XData', model_state(1), 'YData', model_state(2),...
        'UData', arrow_length*cos(model_state(3)), 'VData', arrow_length*sin(model_state(3)));
    set(h_true_orientation, 'XData', true_state(1), 'YData', true_state(2),...
        'UData', arrow_length*cos(true_state(3)), 'VData', arrow_length*sin(true_state(3)));
    set(h_est_orientation,'XData', est_state(1), 'YData', est_state(2),...
        'UData', arrow_length*cos(est_state(3)),'VData', arrow_length*sin(est_state(3)));
    
    % ���ƹ��Ƶĵر�λ��
    delete(h_est_landmarks);
    delete(m_line);
    [h_est_landmarks,m_line] = plot_estimated_landmarks(est_state,landmarks);
    
    % ���ƹ۲�����
    h_obs_lines = update_observation_lines(true_state, landmarks, visible_landmarks, h_obs_lines);
    
    drawnow;
    pause(0.001);
end

%% ������������RMSE
for t = 1:sim_time 
    % ����������
    rmse_true_EKF(t) = sqrt(mean((true_path(:,t) - est_path(:,t)).^2));
    rmse_true_model(t) = sqrt(mean((true_path(:,t) - model_path(:,t)).^2));
end

figure; hold on; grid on; 
plot(1:sim_time, rmse_true_EKF, 'r-o','DisplayName', 'EKF-error','LineWidth', 2);
plot(1:sim_time, rmse_true_model, 'b-o','DisplayName', 'model-error','LineWidth', 2);
xlabel('epoch'); ylabel('REMS(m)'); 
title('��ʵ·�������·�����');
legend('Location', 'best');

EKF_error_mean = mean(rmse_true_EKF);
disp("landmark����Ϊ��");
disp(num_landmarks);
disp("EKF����·��ƽ��RMSE���Ϊ��");
disp(EKF_error_mean);