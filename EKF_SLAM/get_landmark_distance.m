function [d] = get_landmark_distance(state,landmarks)
    d = [];
    num_landmarks = (length(state)-3)/2;
    for i = 1:num_landmarks
        if state(3+2*i-1) ~= 0 || state(3+2*i) ~= 0
            d = [d,sqrt((landmarks(i,1)-state(3+2*i-1))^2+(landmarks(i,2)-state(3+2*i))^2)];
        end
    end
end

