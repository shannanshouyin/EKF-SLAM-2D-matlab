function [state, P] = Iekf_update_B(state, P, measurements, R)
    max_iters = 5; % 默认迭代次数

    if isempty(measurements)
        return;
    end
     
    % 提取所有观测数据
    landmark_indices = measurements(:, 1); % 所有地标的索引
    z_all = measurements(:, 2:3)';         % 所有观测值（距离和角度）

    % IEKF 迭代优化
    for iter = 1:max_iters
        % 初始化联合雅可比矩阵和观测残差
        H_all = [];
        z_pred_all = [];
        z_all_flat = [];

        % 遍历所有观测数据
        for k = 1:size(measurements, 1)
            idx = landmark_indices(k); % 当前地标索引
            z = z_all(:, k);          % 当前观测值

            % 如果地标尚未初始化
            if state(3+2*idx-1) == 0 && state(3+2*idx) == 0
                state(3+2*idx-1:3+2*idx) = [state(1) + z(1)*cos(state(3)+z(2));
                                             state(2) + z(1)*sin(state(3)+z(2))];
            end

            % 计算预期观测值
            dx = state(3+2*idx-1) - state(1);
            dy = state(3+2*idx) - state(2);
            q = dx^2 + dy^2;
            z_pred = [sqrt(q); atan2(dy, dx) - state(3)];

            % 计算雅可比矩阵 H
            H = zeros(2, length(state));
            H(:, 1:3) = [-dx/sqrt(q), -dy/sqrt(q), 0;
                         dy/q,        -dx/q,      -1];
            H(:, 3+2*idx-1:3+2*idx) = [dx/sqrt(q), dy/sqrt(q);
                                       -dy/q,      dx/q];

            % 累积联合雅可比矩阵和观测残差
            H_all = [H_all; H];
            z_pred_all = [z_pred_all; z_pred];
            z_all_flat = [z_all_flat; z];
        end

        % 计算卡尔曼增益
        S = H_all * P * H_all' + kron(eye(size(measurements, 1)), R);
        K = P * H_all' / S;

        % 计算观测残差并角度归一化
        innovation = z_all_flat - z_pred_all;
        innovation(2:2:end) = wrapToPi(innovation(2:2:end)); % 角度归一化

        % 更新状态和协方差矩阵
        state = state + K * innovation;
        P = (eye(size(P)) - K * H_all) * P;
    end
end
