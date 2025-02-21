clear; close all; clc;

%% ��������
num_landmarks = 100;        % �ر�����
max_range = 5;             % ���������۲ⷶΧ
sim_time = 500;             % �ܷ���ʱ��
dt = 0.5;                  % ʱ�䲽��
process_noise = diag([0.01, 0.0001]);       	 % ��������
measure_noise = diag([0.15^2, (pi/180)^2]);  % �۲�����
motion = [1; 0.1];         % �������� [�ٶ�; ת����ٶ�]

%% ��ʼ��
% ��������ر꣨���ȷֲ��ڳ����У�
%rng(1); % �̶�������ӱ�֤���ظ���
true_landmarks_x = 30 * rand(num_landmarks,1) - 15; % ������ɵر�����
landmarks = [true_landmarks_x,20 * rand(num_landmarks,1)];

% ��ʼ��״̬
model_state = [0; 0; 0];     % ����״̬
true_state = [0; 0; 0];      % ��ʵ״̬ [x, y, theta]
est_state = [true_state; zeros(2 * num_landmarks,1)]; % ����״̬�������ر꣩
covariance = eye(3 + 2*num_landmarks); % Э�������
first_obs = zeros(2*num_landmarks+3,2);
est_state_pre = [0;0;0];

est_state_I = [true_state; zeros(2 * num_landmarks,1)]; % ����״̬�������ر꣩
covariance_I = eye(3 + 2*num_landmarks); % Э�������
first_obs_I = struct('pos', zeros(size(est_state,1),1), 'robot_pose', zeros(3,1),...
                    'initialized', false((size(est_state,1)-3)/2,1));
est_state_pre_I = [0;0;0];

% �洢��ʷ�켣
model_path = model_state(1:3);
true_path = true_state(1:3);
est_path = est_state(1:3);
est_path_I = est_state(1:3);

% % ͼ�γ�ʼ��
% figure; hold on; grid on; axis equal;
% % ����·��
% h_model = plot(model_state(1), model_state(2), 'g.-', 'DisplayName', 'ģ��·��','LineWidth', 2);
% h_actual = plot(true_state(1), true_state(2), 'b.-', 'DisplayName', 'ʵ��·��','LineWidth', 2);
% h_est = plot(est_state(1), est_state(2), 'r.-', 'DisplayName', '����·��','LineWidth', 2);
% 
% % ���Ƴ����ͷ
% arrow_length = 2;
% h_model_orientation = quiver(model_state(1), model_state(2),arrow_length*cos(model_state(3)),...
%                            arrow_length*sin(model_state(3)),'g', 'LineWidth', 2, 'MaxHeadSize', 1, 'DisplayName', 'ģ�ͳ���');
% h_true_orientation = quiver(true_state(1), true_state(2),arrow_length*cos(true_state(3)),...
%                            arrow_length*sin(true_state(3)),'b', 'LineWidth', 2, 'MaxHeadSize', 1, 'DisplayName', '��ʵ����');
% h_est_orientation = quiver(est_state(1), est_state(2),arrow_length*cos(est_state(3)),...
%                           arrow_length*sin(est_state(3)), 'r', 'LineWidth', 2, 'MaxHeadSize', 1, 'DisplayName', '���Ƴ���');
% 
% % ���ƻ���ͼ��                      
% h_car = plot(true_state(1), true_state(2), 'ro', 'MarkerSize',8, 'LineWidth',2);
% h_range = rectangle('Position',[true_state(1)-max_range, true_state(2)-max_range,...
%                  2*max_range, 2*max_range], 'Curvature',[1,1], 'EdgeColor','b');
% h_landmarks = plot(landmarks(:,1), landmarks(:,2), 'k^', 'MarkerSize',8,...
%                  'DisplayName', '��ʵ�ر�');
%              
% h_obs_lines = gobjects(0); % �洢�۲����ߵ�ͼ�ξ��
% h_est_landmarks = gobjects(0); % ���Ƶر��ͼ�ξ��
% m_line = gobjects(0); % ���Ƶر��ͼ�ξ��
% legend('Location', 'best'); 
% xlabel('X'); ylabel('Y'); 
% title('EKF-SLAM ����');

%% ��ѭ��
for t = 0:dt:sim_time
    %% ��ʵϵͳ�˶���Բ���˶���
    model_state = motion_model(model_state, motion, dt);
    true_state = motion_model_noise(true_state, motion, dt, process_noise);
    
    %% EKFԤ�ⲽ��
    [est_state, covariance] = fekf_predict(est_state, est_state_pre, covariance, motion, dt, process_noise);
    est_state_pre = est_state(1:3);
    [est_state_I, covariance_I] = fekf_predict(est_state_I, est_state_pre_I, covariance_I, motion, dt, process_noise);
    est_state_pre_I = est_state_I(1:3);
    %% �۲⴦��
    visible_landmarks = find_visible_landmarks(true_state, landmarks, max_range);% ���ص���landmarks������
    measurements = get_measurements(true_state, landmarks, visible_landmarks, measure_noise);
 
    %% EKF���²���
    [est_state, P, first_obs] = fekf_update(est_state, covariance ,measurements, measure_noise, first_obs);
    [est_state_I, P_I, first_obs_I] = f_Iekf_update_B(est_state_I, covariance_I ,measurements, measure_noise, first_obs_I);

    %% ��¼�켣
    model_path = [model_path, model_state(1:3)];
    true_path = [true_path, true_state(1:3)];
    est_path = [est_path, est_state(1:3)];
    est_path_I = [est_path_I, est_state_I(1:3)];
%     %% ��̬���ӻ�
%     % ����С��λ�ú͹۲ⷶΧ
%     set(h_car, 'XData', true_state(1), 'YData', true_state(2));
%     set(h_range, 'Position', [true_state(1)-max_range, true_state(2)-max_range,...
%                               2*max_range, 2*max_range]);
%     
%     % ����·��
%     set(h_model, 'XData', model_path(1,:), 'YData', model_path(2,:));
%     set(h_actual, 'XData', true_path(1,:), 'YData', true_path(2,:));
%     set(h_est, 'XData', est_path(1,:), 'YData', est_path(2,:));
%     
%     % ���³����ͷ
%     set(h_model_orientation, 'XData', model_state(1), 'YData', model_state(2),...
%         'UData', arrow_length*cos(model_state(3)), 'VData', arrow_length*sin(model_state(3)));
%     set(h_true_orientation, 'XData', true_state(1), 'YData', true_state(2),...
%         'UData', arrow_length*cos(true_state(3)), 'VData', arrow_length*sin(true_state(3)));
%     set(h_est_orientation,'XData', est_state(1), 'YData', est_state(2),...
%         'UData', arrow_length*cos(est_state(3)),'VData', arrow_length*sin(est_state(3)));
%     
%     % ���ƹ��Ƶĵر�λ��
%     delete(h_est_landmarks);
%     delete(m_line);
%     [h_est_landmarks,m_line] = plot_estimated_landmarks(est_state,landmarks);
%     
%     % ���ƹ۲�����
%     h_obs_lines = update_observation_lines(true_state, landmarks, visible_landmarks, h_obs_lines);
%     
%     drawnow;
%     pause(0.001);
end

%% ������������RMSE
for t = 1:sim_time 
    % ����������
    rmse_true_FEKF(t) = sqrt(mean((true_path(:,t) - est_path(:,t)).^2));
    rmse_true_FEKF_Pos(t) = sqrt(mean((true_path(1:2,t) - est_path(1:2,t)).^2));
    rmse_true_FEKF_Head(t) = sqrt(mean((true_path(3,t) - est_path(3,t)).^2));
    
    rmse_true_FIEKF(t) = sqrt(mean((true_path(:,t) - est_path_I(:,t)).^2));
    rmse_true_FIEKF_Pos(t) = sqrt(mean((true_path(1:2,t) - est_path_I(1:2,t)).^2));
    rmse_true_FIEKF_Head(t) = sqrt(mean((true_path(3,t) - est_path_I(3,t)).^2));

end

figure; hold on;
plot(1:sim_time, rmse_true_FEKF, 'r-o','DisplayName', 'FEJ-EKF-Pose-RMSE','LineWidth', 2);
plot(1:sim_time, rmse_true_FIEKF, 'b-o','DisplayName', 'FEJ-IEKF-Pose-RMSE','LineWidth', 2);
xlabel('time'); ylabel('RMSE(m)'); 
title('FEJ-EKF��FEJ-IEKFλ�˹���RMSE�Ա�');
legend('Location', 'northeast');

figure; hold on; grid on; 
subplot(2,1,1);
plot(1:sim_time, rmse_true_FEKF_Pos, 'r-o','DisplayName', 'FEJ-EKF-Position-RMSE','LineWidth', 2);
hold on;
plot(1:sim_time, rmse_true_FIEKF_Pos, 'b-o','DisplayName', 'FEJ-IEKF-Position-RMSE','LineWidth', 2);
xlabel('time'); ylabel('RMSE(m)'); 
title('FEJ-EKF��FEJ-IEKF�������RMSE�Ա�');
legend('Location', 'northeast');
subplot(2,1,2);
plot(1:sim_time, rmse_true_FEKF_Head, 'r-o','DisplayName', 'FEJ-EKF-Heading-RMSE','LineWidth', 2);
hold on;
plot(1:sim_time, rmse_true_FIEKF_Head, 'b-o','DisplayName', 'FEJ-IEKF-Heading-RMSE','LineWidth', 2);
xlabel('time'); ylabel('RMSE(m)'); 
title('FEJ-EKF��FEJ-IEKF�������RMSE�Ա�');
legend('Location', 'northeast');


FEKF_error_mean = mean(rmse_true_FEKF);
FIEKF_error_mean = mean(rmse_true_FIEKF);
disp("F-EKF����λ��ƽ��RMSE���Ϊ��");
disp(FEKF_error_mean);
disp("F-IEKF����λ��ƽ��RMSE���Ϊ��");
disp(FIEKF_error_mean);

FEKF_error_Pos = mean(rmse_true_FEKF_Pos);
FIEKF_error_Pos = mean(rmse_true_FIEKF_Pos);
disp("F-EKF��������ƽ��RMSE���Ϊ��");
disp(FEKF_error_Pos);
disp("F-IEKF��������ƽ��RMSE���Ϊ��");
disp(FIEKF_error_Pos);

FEKF_error_head = mean(rmse_true_FEKF_Head);
FIEKF_error_head = mean(rmse_true_FIEKF_Head);
disp("F-EKF���Ƴ���ƽ��RMSE���Ϊ��");
disp(FEKF_error_head);
disp("F-IEKF���Ƴ���ƽ��RMSE���Ϊ��");
disp(FIEKF_error_head);

%% �ر�������
d_land = get_landmark_distance(est_state,landmarks);
d_land_I = get_landmark_distance(est_state_I,landmarks);

FEKF_derror_mean = mean(d_land);
FIEKF_derror_mean = mean(d_land_I);

disp("FEKF��ͼƽ��RMSEΪ��");
disp(FEKF_derror_mean);
disp("FIEKF��ͼƽ��RMSEΪ��");
disp(FIEKF_derror_mean);

figure; hold on;  
plot(d_land,'r-o','DisplayName','FEJ-EFK-landmark-Error','LineWidth', 2);
plot(d_land_I,'b-o','DisplayName','FEJ-IEFK-landmark-Error','LineWidth', 2);
xlabel('landmarks'); ylabel('RMSE(m)'); 
title('FEJ-EKF��FEJ-IEKF�ر����RMSE�Ա�');
legend('Location', 'northeast');