% 更新观测连线
function [h_lines] = update_observation_lines(state, landmarks, visible, h_lines)
    % 删除旧的连线
    delete(h_lines);
    h_lines = [];
    
    % 绘制新连线
    for i = 1:length(visible)
        idx = visible(i);
        h_lines = [h_lines, plot([state(1), landmarks(idx,1)],...
                                 [state(2), landmarks(idx,2)], 'm--','HandleVisibility', 'off')];
    end
end
