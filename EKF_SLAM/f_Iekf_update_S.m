function [state, P, first_obs] = f_Iekf_update_S(state, P, measurements, R, first_obs)
% ����state��״̬�ռ��������ֵ    
%     P��Э�������    
%     measurements��n��3�й۲��������Ϊ��ǰʱ�̹۲⵽��landmark������һ��Ϊlandmark������������Ϊ��������
%     R���۲�����Э�������
% ���state�����º��״̬�ռ�
%     P�����º��Э�������

    max_iter = 5; % ����������
    epsilon = 1e-3; % ������ֵ

    for k = 1:size(measurements,1)
        idx = measurements(k,1);
        z = measurements(k,2:3)';
        
        % ����ر���δ��ʼ��
        if state(3+2*idx-1) == 0 && state(3+2*idx) == 0
           state(3+2*idx-1:3+2*idx) = [state(1) + z(1)*cos(state(3)+z(2));
                                       state(2) + z(1)*sin(state(3)+z(2))];
           first_obs(3+2*idx-1:3+2*idx,1) = state(3+2*idx-1:3+2*idx);
           first_obs(3+2*idx-1:3+2*idx,2) = [state(1);state(2)];
        end

        iter = 0;
        delta_state = inf;
        state_iter = state; % ��ʼ������״̬

        while iter < max_iter && norm(delta_state) > epsilon
            % ����Ԥ�ڲ���ֵ
            dx = state_iter(3+2*idx-1) - state_iter(1);
            dy = state_iter(3+2*idx) - state_iter(2);
            q = dx^2 + dy^2;
            z_pred = [sqrt(q); atan2(dy, dx) - state_iter(3)];

            % �ſɱȾ���
            dx = first_obs(3+2*idx-1) - state_iter(1);
            dy = first_obs(3+2*idx) - state_iter(2);
            q = dx^2 + dy^2;
            H = zeros(2, length(state));
            H(:,1:3) = [-dx/sqrt(q), -dy/sqrt(q), 0;
                        dy/q,        -dx/q,      -1];
            H(:,3+2*idx-1:3+2*idx) = [dx/sqrt(q), dy/sqrt(q);
                                      -dy/q,      dx/q];

            % ����������
            K = P*H'*pinv(H*P*H' + R);

            % ״̬����
            innovation = z - z_pred;
            innovation(2) = wrapToPi(innovation(2)); % �Ƕȹ�һ��
            delta_state = K*innovation;
            state_iter = state_iter + delta_state;

            iter = iter + 1;
        end

        % ����״̬
        state = state_iter;

        % ���¼����ſɱȾ�������Э�������
        dx = first_obs(3+2*idx-1,1) - state(1);
        dy = first_obs(3+2*idx,1) - state(2);
        q = dx^2 + dy^2;
        H = zeros(2, length(state));
        H(:,1:3) = [-dx/sqrt(q), -dy/sqrt(q), 0;
                    dy/q,        -dx/q,      -1];
        H(:,3+2*idx-1:3+2*idx) = [dx/sqrt(q), dy/sqrt(q);
                                  -dy/q,      dx/q];

        % Э�������
        P = (eye(size(P)) - K*H)*P;
    end
end
