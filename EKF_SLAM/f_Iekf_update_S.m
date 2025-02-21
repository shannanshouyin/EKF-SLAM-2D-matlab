function [state, P, first_obs] = f_Iekf_update_S(state, P, measurements, R, first_obs)
% 输入state：状态空间先验估计值    
%     P：协方差矩阵    
%     measurements：n行3列观测矩阵，行数为当前时刻观测到的landmark数，第一列为landmark索引，二三列为距离和向角
%     R：观测噪声协方差矩阵
% 输出state：更新后的状态空间
%     P：更新后的协方差矩阵

    max_iter = 5; % 最大迭代次数
    epsilon = 1e-3; % 收敛阈值

    for k = 1:size(measurements,1)
        idx = measurements(k,1);
        z = measurements(k,2:3)';
        
        % 如果地标尚未初始化
        if state(3+2*idx-1) == 0 && state(3+2*idx) == 0
           state(3+2*idx-1:3+2*idx) = [state(1) + z(1)*cos(state(3)+z(2));
                                       state(2) + z(1)*sin(state(3)+z(2))];
           first_obs(3+2*idx-1:3+2*idx,1) = state(3+2*idx-1:3+2*idx);
           first_obs(3+2*idx-1:3+2*idx,2) = [state(1);state(2)];
        end

        iter = 0;
        delta_state = inf;
        state_iter = state; % 初始化迭代状态

        while iter < max_iter && norm(delta_state) > epsilon
            % 计算预期测量值
            dx = state_iter(3+2*idx-1) - state_iter(1);
            dy = state_iter(3+2*idx) - state_iter(2);
            q = dx^2 + dy^2;
            z_pred = [sqrt(q); atan2(dy, dx) - state_iter(3)];

            % 雅可比矩阵
            dx = first_obs(3+2*idx-1) - state_iter(1);
            dy = first_obs(3+2*idx) - state_iter(2);
            q = dx^2 + dy^2;
            H = zeros(2, length(state));
            H(:,1:3) = [-dx/sqrt(q), -dy/sqrt(q), 0;
                        dy/q,        -dx/q,      -1];
            H(:,3+2*idx-1:3+2*idx) = [dx/sqrt(q), dy/sqrt(q);
                                      -dy/q,      dx/q];

            % 卡尔曼增益
            K = P*H'*pinv(H*P*H' + R);

            % 状态更新
            innovation = z - z_pred;
            innovation(2) = wrapToPi(innovation(2)); % 角度归一化
            delta_state = K*innovation;
            state_iter = state_iter + delta_state;

            iter = iter + 1;
        end

        % 更新状态
        state = state_iter;

        % 重新计算雅可比矩阵用于协方差更新
        dx = first_obs(3+2*idx-1,1) - state(1);
        dy = first_obs(3+2*idx,1) - state(2);
        q = dx^2 + dy^2;
        H = zeros(2, length(state));
        H(:,1:3) = [-dx/sqrt(q), -dy/sqrt(q), 0;
                    dy/q,        -dx/q,      -1];
        H(:,3+2*idx-1:3+2*idx) = [dx/sqrt(q), dy/sqrt(q);
                                  -dy/q,      dx/q];

        % 协方差更新
        P = (eye(size(P)) - K*H)*P;
    end
end
