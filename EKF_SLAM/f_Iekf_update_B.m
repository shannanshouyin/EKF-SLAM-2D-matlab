% function [state, P, first_obs] = f_Iekf_update_B(state, P, measurements, R, first_obs)
% % ����state��״̬�ռ��������ֵ    
% %     P��Э�������    
% %     measurements��n��3�й۲��������Ϊ��ǰʱ�̹۲⵽��landmark������һ��Ϊlandmark������������Ϊ��������
% %     R���۲�����Э�������
% %     first_obs�������һ�ι۲⵽landmark����Ϣ
% % ���state�����º��״̬�ռ�
% %     P�����º��Э�������
%     max_iters = 5; % Ĭ�ϵ�������
%     if isempty(measurements)
%         return;
%     end
%      
%     % ��ȡ���й۲�����
%     landmark_indices = measurements(:, 1); % ���еر������
%     z_all = measurements(:, 2:3)';         % ���й۲�ֵ������ͽǶȣ�
% 
%     % IEKF �����Ż�
%     for iter = 1:max_iters
%         % ��ʼ�������ſɱȾ���͹۲�в�
%         H_all = [];
%         z_pred_all = [];
%         z_all_flat = [];
% 
%         % �������й۲�����
%         for k = 1:size(measurements, 1)
%             idx = landmark_indices(k); % ��ǰ�ر�����
%             z = z_all(:, k);          % ��ǰ�۲�ֵ
% 
%             % ����ر���δ��ʼ��
%             if state(3+2*idx-1) == 0 && state(3+2*idx) == 0
%                state(3+2*idx-1:3+2*idx) = [state(1) + z(1)*cos(state(3)+z(2));
%                                              state(2) + z(1)*sin(state(3)+z(2))];
%                first_obs(3+2*idx-1:3+2*idx,1) = state(3+2*idx-1:3+2*idx);
%                first_obs(3+2*idx-1:3+2*idx,2) = [state(1);state(2)];
%             end
% 
%             % ����Ԥ�ڹ۲�ֵ
%             dx = state(3+2*idx-1) - state(1);
%             dy = state(3+2*idx) - state(2);
%             q = dx^2 + dy^2;
%             z_pred = [sqrt(q); atan2(dy, dx) - state(3)];
% 
%             % �����ſɱȾ��� H
%             dx = first_obs(3+2*idx-1,1) - state(1);
%             dy = first_obs(3+2*idx,1) - state(2);
%             q = dx^2 + dy^2;
%             
%             H = zeros(2, length(state));
%             H(:, 1:3) = [-dx/sqrt(q), -dy/sqrt(q), 0;
%                          dy/q,        -dx/q,      -1];
%             H(:, 3+2*idx-1:3+2*idx) = [dx/sqrt(q), dy/sqrt(q);
%                                        -dy/q,      dx/q];
% 
%             % �ۻ������ſɱȾ���͹۲�в�
%             H_all = [H_all; H];
%             z_pred_all = [z_pred_all; z_pred];
%             z_all_flat = [z_all_flat; z];
%         end
% 
%         % ���㿨��������
%         S = H_all * P * H_all' + kron(eye(size(measurements, 1)), R);
%         K = P * H_all' / S;
% 
%         % ����۲�в�Ƕȹ�һ��
%         innovation = z_all_flat - z_pred_all;
%         innovation(2:2:end) = wrapToPi(innovation(2:2:end)); % �Ƕȹ�һ��
%         state = state + K * innovation;
%         P = (eye(size(P)) - K * H_all) * P;
%     end
% end


function [state, P, first_obs] = f_Iekf_update_B(state, P, measurements, R, first_obs)
    max_iters = 5;
    if isempty(measurements)
        return;
    end
    
    landmark_indices = measurements(:, 1);
    z_all = measurements(:, 2:3)';

    % ��ʼ��first_obs�ṹ�������һ������ʱ��Ҫ��ʼ����
    if isempty(first_obs)
        % first_obs�ṹ: [�ر�λ��(��1), �����˳�ʼλ��(��2), ��ʼ����־λ(��3)]
        first_obs = struct('pos', zeros(size(state,1),1), 'robot_pose', zeros(3,1), 'initialized', false(size(state,1)/2,1));
    end

    for iter = 1:max_iters
        H_all = [];
        z_pred_all = [];
        z_all_flat = [];

        for k = 1:size(measurements, 1)
            idx = landmark_indices(k);
            z = z_all(:, k);
            
            % ���ر��Ƿ��ѳ�ʼ��
            if ~first_obs.initialized(idx)
                % ʹ���״ι۲��ʼ���ر�
                x_robot_init = state(1);
                y_robot_init = state(2);
                theta_robot_init = state(3);
                
                mx = x_robot_init + z(1)*cos(theta_robot_init + z(2));
                my = y_robot_init + z(1)*sin(theta_robot_init + z(2));
                
                % ����״̬��first_obs
                state(3+2*idx-1:3+2*idx) = [mx; my];
                first_obs.pos(3+2*idx-1:3+2*idx, 1) = [mx; my];
                first_obs.robot_pose(:, idx) = [x_robot_init; y_robot_init; theta_robot_init];
                first_obs.initialized(idx) = true;
            end

            % FEJ��ʹ���״ι۲�ʱ�Ļ�����λ�˺͵ر�λ��
            x_init = first_obs.robot_pose(1, idx);
            y_init = first_obs.robot_pose(2, idx);
            mx = first_obs.pos(3+2*idx-1);
            my = first_obs.pos(3+2*idx);

            % ����Ԥ�ڹ۲�ֵ�����ڵ�ǰ״̬��
            dx_curr = state(3+2*idx-1) - state(1);
            dy_curr = state(3+2*idx) - state(2);
            q_curr = dx_curr^2 + dy_curr^2;
            z_pred = [sqrt(q_curr); atan2(dy_curr, dx_curr) - state(3)];

            % FEJ�����ſɱȣ������״�λ�ˣ�
            dx_fej = mx - state(1);
            dy_fej = my - state(2);
            q_fej = dx_fej^2 + dy_fej^2;
            
            H = zeros(2, length(state));
            % �Ի�����λ�˵��ſɱ�
            H(:, 1:3) = [-dx_fej/sqrt(q_fej), -dy_fej/sqrt(q_fej), 0;
                         dy_fej/q_fej,        -dx_fej/q_fej,      -1];
            % �ر�λ�ò����£��ſɱȱ���Ϊ0
            % H(:, 3+2*idx-1:3+2*idx) = [0, 0; 0, 0]; 

            H_all = [H_all; H];
            z_pred_all = [z_pred_all; z_pred];
            z_all_flat = [z_all_flat; z];
        end

        S = H_all * P * H_all' + kron(eye(size(measurements, 1)), R);
        K = P * H_all' / S;
        
        innovation = z_all_flat - z_pred_all;
        innovation(2:2:end) = wrapToPi(innovation(2:2:end));
        
        % �����»�����λ�ˣ�ǰ3��״̬��
        state(1:3) = state(1:3) + K(1:3, :) * innovation;
        P = (eye(size(P)) - K * H_all) * P;
    end
end
