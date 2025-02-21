% EKF���²���
function [state, P] = ekf_update_test(state, P, measurements, R)
% ����state��״̬�ռ��������ֵ    
%     P��Э�������    
%     measurements��n��3�й۲��������Ϊ��ǰʱ�̹۲⵽��landmark������һ��Ϊlandmark������������Ϊ��������
%     R���۲�����Э�������
% ���state�����º��״̬�ռ�
%     P�����º��Э�������
    for k = 1:size(measurements,1)
        idx = measurements(k,1);
        z = measurements(k,2:3)';
        
        H_all = [];
        z_pred_all = [];
        z_all_flat = [];
        
        % ����ر���δ��ʼ��
        if state(3+2*idx-1) == 0 && state(3+2*idx) == 0
           state(3+2*idx-1:3+2*idx) = [state(1) + z(1)*cos(state(3)+z(2));
                                       state(2) + z(1)*sin(state(3)+z(2))];
        end
        
        % ����Ԥ�ڲ���ֵ
        dx = state(3+2*idx-1) - state(1);
        dy = state(3+2*idx) - state(2);
        q = dx^2 + dy^2;
        z_pred = [sqrt(q); atan2(dy, dx) - state(3)];
        
        % �ſɱȾ���
        H = zeros(2, length(state));
        H(:,1:3) = [-dx/sqrt(q), -dy/sqrt(q), 0;
                    dy/q,        -dx/q,      -1];
        H(:,3+2*idx-1:3+2*idx) = [dx/sqrt(q), dy/sqrt(q);
                                  -dy/q,      dx/q];
                              
        % �ۻ������ſɱȾ���͹۲�в�
        H_all = [H_all; H];
        z_pred_all = [z_pred_all; z_pred];
        z_all_flat = [z_all_flat; z];                     
    end
    
    % ����������
    K = P*H'/(H*P*H' + R);

    % ״̬����
    innovation = z - z_pred;
    innovation(2) = wrapToPi(innovation(2)); % �Ƕȹ�һ��
    state = state + K*innovation;
    P = (eye(size(P)) - K*H)*P;
end
