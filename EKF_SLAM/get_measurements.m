% 获取测量值
function measurements = get_measurements(state, landmarks, visible, R)
    measurements = [];
    for i = 1:length(visible)
        idx = visible(i);
        dx = landmarks(idx,1) - state(1);
        dy = landmarks(idx,2) - state(2);
        
        % 添加噪声
        noise = chol(R)' * randn(2,1);
        range = sqrt(dx^2 + dy^2) + noise(1);%距离
        bearing = atan2(dy, dx) - state(3) + noise(2);%角度
        
        measurements = [measurements; idx, range, bearing];
    end
end

