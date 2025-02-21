% EKF更新步骤
function [state, P] = ekf_update_test(state, P, measurements, R)
% 输入state：状态空间先验估计值    
%     P：协方差矩阵    
%     measurements：n行3列观测矩阵，行数为当前时刻观测到的landmark数，第一列为landmark索引，二三列为距离和向角
%     R：观测噪声协方差矩阵
% 输出state：更新后的状态空间
%     P：更新后的协方差矩阵
    for k = 1:size(measurements,1)
        idx = measurements(k,1);
        z = measurements(k,2:3)';
        
        H_all = [];
        z_pred_all = [];
        z_all_flat = [];
        
        % 如果地标尚未初始化
        if state(3+2*idx-1) == 0 && state(3+2*idx) == 0
           state(3+2*idx-1:3+2*idx) = [state(1) + z(1)*cos(state(3)+z(2));
                                       state(2) + z(1)*sin(state(3)+z(2))];
        end
        
        % 计算预期测量值
        dx = state(3+2*idx-1) - state(1);
        dy = state(3+2*idx) - state(2);
        q = dx^2 + dy^2;
        z_pred = [sqrt(q); atan2(dy, dx) - state(3)];
        
        % 雅可比矩阵
        H = zeros(2, length(state));
        H(:,1:3) = [-dx/sqrt(q), -dy/sqrt(q), 0;
                    dy/q,        -dx/q,      -1];
        H(:,3+2*idx-1:3+2*idx) = [dx/sqrt(q), dy/sqrt(q);
                                  -dy/q,      dx/q];
                              
        % 累积联合雅可比矩阵和观测残差
        H_all = [H_all; H];
        z_pred_all = [z_pred_all; z_pred];
        z_all_flat = [z_all_flat; z];                     
    end
    
    % 卡尔曼增益
    K = P*H'/(H*P*H' + R);

    % 状态更新
    innovation = z - z_pred;
    innovation(2) = wrapToPi(innovation(2)); % 角度归一化
    state = state + K*innovation;
    P = (eye(size(P)) - K*H)*P;
end
