% 绘制估计地标
function [h,m] = plot_estimated_landmarks_F(state,landmarks)
    h = [];
    m = [];
    num_landmarks = (length(state)-3)/2;
    for i = 1:num_landmarks
        if state(3+2*i-1) ~= 0 || state(3+2*i) ~= 0
            h = [h, plot(state(3+2*i-1), state(3+2*i), 'm*', 'MarkerSize', 8,'HandleVisibility', 'off')];
            m = [m,plot([landmarks(i,1),state(3+2*i-1)],[landmarks(i,2),state(3+2*i)], 'm--','HandleVisibility', 'off')];
        end
    end
end

