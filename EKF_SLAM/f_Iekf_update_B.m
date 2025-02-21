% function [state, P, first_obs] = f_Iekf_update_B(state, P, measurements, R, first_obs)
% % 输入state：状态空间先验估计值    
% %     P：协方差矩阵    
% %     measurements：n行3列观测矩阵，行数为当前时刻观测到的landmark数，第一列为landmark索引，二三列为距离和向角
% %     R：观测噪声协方差矩阵
% %     first_obs：储存第一次观测到landmark的信息
% % 输出state：更新后的状态空间
% %     P：更新后的协方差矩阵
%     max_iters = 5; % 默认迭代次数
%     if isempty(measurements)
%         return;
%     end
%      
%     % 提取所有观测数据
%     landmark_indices = measurements(:, 1); % 所有地标的索引
%     z_all = measurements(:, 2:3)';         % 所有观测值（距离和角度）
% 
%     % IEKF 迭代优化
%     for iter = 1:max_iters
%         % 初始化联合雅可比矩阵和观测残差
%         H_all = [];
%         z_pred_all = [];
%         z_all_flat = [];
% 
%         % 遍历所有观测数据
%         for k = 1:size(measurements, 1)
%             idx = landmark_indices(k); % 当前地标索引
%             z = z_all(:, k);          % 当前观测值
% 
%             % 如果地标尚未初始化
%             if state(3+2*idx-1) == 0 && state(3+2*idx) == 0
%                state(3+2*idx-1:3+2*idx) = [state(1) + z(1)*cos(state(3)+z(2));
%                                              state(2) + z(1)*sin(state(3)+z(2))];
%                first_obs(3+2*idx-1:3+2*idx,1) = state(3+2*idx-1:3+2*idx);
%                first_obs(3+2*idx-1:3+2*idx,2) = [state(1);state(2)];
%             end
% 
%             % 计算预期观测值
%             dx = state(3+2*idx-1) - state(1);
%             dy = state(3+2*idx) - state(2);
%             q = dx^2 + dy^2;
%             z_pred = [sqrt(q); atan2(dy, dx) - state(3)];
% 
%             % 计算雅可比矩阵 H
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
%             % 累积联合雅可比矩阵和观测残差
%             H_all = [H_all; H];
%             z_pred_all = [z_pred_all; z_pred];
%             z_all_flat = [z_all_flat; z];
%         end
% 
%         % 计算卡尔曼增益
%         S = H_all * P * H_all' + kron(eye(size(measurements, 1)), R);
%         K = P * H_all' / S;
% 
%         % 计算观测残差并角度归一化
%         innovation = z_all_flat - z_pred_all;
%         innovation(2:2:end) = wrapToPi(innovation(2:2:end)); % 角度归一化
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

    % 初始化first_obs结构（假设第一次运行时需要初始化）
    if isempty(first_obs)
        % first_obs结构: [地标位置(列1), 机器人初始位姿(列2), 初始化标志位(列3)]
        first_obs = struct('pos', zeros(size(state,1),1), 'robot_pose', zeros(3,1), 'initialized', false(size(state,1)/2,1));
    end

    for iter = 1:max_iters
        H_all = [];
        z_pred_all = [];
        z_all_flat = [];

        for k = 1:size(measurements, 1)
            idx = landmark_indices(k);
            z = z_all(:, k);
            
            % 检查地标是否已初始化
            if ~first_obs.initialized(idx)
                % 使用首次观测初始化地标
                x_robot_init = state(1);
                y_robot_init = state(2);
                theta_robot_init = state(3);
                
                mx = x_robot_init + z(1)*cos(theta_robot_init + z(2));
                my = y_robot_init + z(1)*sin(theta_robot_init + z(2));
                
                % 更新状态和first_obs
                state(3+2*idx-1:3+2*idx) = [mx; my];
                first_obs.pos(3+2*idx-1:3+2*idx, 1) = [mx; my];
                first_obs.robot_pose(:, idx) = [x_robot_init; y_robot_init; theta_robot_init];
                first_obs.initialized(idx) = true;
            end

            % FEJ：使用首次观测时的机器人位姿和地标位置
            x_init = first_obs.robot_pose(1, idx);
            y_init = first_obs.robot_pose(2, idx);
            mx = first_obs.pos(3+2*idx-1);
            my = first_obs.pos(3+2*idx);

            % 计算预期观测值（基于当前状态）
            dx_curr = state(3+2*idx-1) - state(1);
            dy_curr = state(3+2*idx) - state(2);
            q_curr = dx_curr^2 + dy_curr^2;
            z_pred = [sqrt(q_curr); atan2(dy_curr, dx_curr) - state(3)];

            % FEJ计算雅可比（基于首次位姿）
            dx_fej = mx - state(1);
            dy_fej = my - state(2);
            q_fej = dx_fej^2 + dy_fej^2;
            
            H = zeros(2, length(state));
            % 对机器人位姿的雅可比
            H(:, 1:3) = [-dx_fej/sqrt(q_fej), -dy_fej/sqrt(q_fej), 0;
                         dy_fej/q_fej,        -dx_fej/q_fej,      -1];
            % 地标位置不更新，雅可比保持为0
            % H(:, 3+2*idx-1:3+2*idx) = [0, 0; 0, 0]; 

            H_all = [H_all; H];
            z_pred_all = [z_pred_all; z_pred];
            z_all_flat = [z_all_flat; z];
        end

        S = H_all * P * H_all' + kron(eye(size(measurements, 1)), R);
        K = P * H_all' / S;
        
        innovation = z_all_flat - z_pred_all;
        innovation(2:2:end) = wrapToPi(innovation(2:2:end));
        
        % 仅更新机器人位姿（前3个状态）
        state(1:3) = state(1:3) + K(1:3, :) * innovation;
        P = (eye(size(P)) - K * H_all) * P;
    end
end
