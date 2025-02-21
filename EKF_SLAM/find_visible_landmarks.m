% 寻找可见地标
function visible = find_visible_landmarks(state, landmarks, max_range)
    dx = landmarks(:,1) - state(1);
    dy = landmarks(:,2) - state(2);
    distances = sqrt(dx.^2 + dy.^2);
    visible = find(distances < max_range);
end

