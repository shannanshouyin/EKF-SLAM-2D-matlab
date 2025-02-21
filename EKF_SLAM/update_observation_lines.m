% ���¹۲�����
function [h_lines] = update_observation_lines(state, landmarks, visible, h_lines)
    % ɾ���ɵ�����
    delete(h_lines);
    h_lines = [];
    
    % ����������
    for i = 1:length(visible)
        idx = visible(i);
        h_lines = [h_lines, plot([state(1), landmarks(idx,1)],...
                                 [state(2), landmarks(idx,2)], 'm--','HandleVisibility', 'off')];
    end
end
