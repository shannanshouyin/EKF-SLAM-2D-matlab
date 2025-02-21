% FEKF更新步骤
function [state, P ,first_obs] = fekf_update(state, P, measurements, R, first_obs)
    % first_obs为一个2*n+3行2列的矩阵，第一列保存首次观测到landmark的坐标，第二列保存对应首次观测到landmark时小车的坐标
    for k = 1:size(measurements,1)
        idx = measurements(k,1);
        z = measurements(k,2:3)';
        % 遇到新地标时
        if state(3+2*idx-1) == 0 && state(3+2*idx) == 0
           state(3+2*idx-1:3+2*idx) = [state(1) + z(1)*cos(state(3)+z(2));
                                       state(2) + z(1)*sin(state(3)+z(2))];                            
           first_obs(3+2*idx-1:3+2*idx,1) = state(3+2*idx-1:3+2*idx);
           first_obs(3+2*idx-1:3+2*idx,2) = [state(1);state(2)];                         
        end     
%% 计算预期测量值
        dx = state(3+2*idx-1) - state(1);
        dy = state(3+2*idx) - state(2);
        q = dx^2 + dy^2;
        z_pred = [sqrt(q);atan2(dy, dx) - state(3)];

%% 计算H矩阵
%         dx = first_obs(3+2*idx-1,1) - first_obs(3+2*idx-1,2);
%         dy = first_obs(3+2*idx,1) - first_obs(3+2*idx,2);
%         q = dx^2 + dy^2;

        dx = first_obs(3+2*idx-1,1) - state(1);
        dy = first_obs(3+2*idx,1) - state(2);
        q = dx^2 + dy^2;
        
        % 雅可比矩阵
        H = zeros(2, length(state));
        H(:,1:3) = [-dx/sqrt(q), -dy/sqrt(q), 0;
                    dy/q,        -dx/q,      -1];
        H(:,3+2*idx-1:3+2*idx) = [dx/sqrt(q), dy/sqrt(q);
                                  -dy/q,      dx/q];                          
    %% 更新计算                              
        % 卡尔曼增益
        K = P*H'/(H*P*H' + R);
        % 状态更新
        innovation = z - z_pred;
        innovation(2) = wrapToPi(innovation(2)); % 角度归一化
        state = state + K*innovation;
        P = (eye(size(P)) - K*H) * P;
    end
end

