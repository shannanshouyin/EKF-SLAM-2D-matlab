function [state, P] = Iekf_update_B(state, P, measurements, R)
    max_iters = 5; % Ĭ�ϵ�������

    if isempty(measurements)
        return;
    end
     
    % ��ȡ���й۲�����
    landmark_indices = measurements(:, 1); % ���еر������
    z_all = measurements(:, 2:3)';         % ���й۲�ֵ������ͽǶȣ�

    % IEKF �����Ż�
    for iter = 1:max_iters
        % ��ʼ�������ſɱȾ���͹۲�в�
        H_all = [];
        z_pred_all = [];
        z_all_flat = [];

        % �������й۲�����
        for k = 1:size(measurements, 1)
            idx = landmark_indices(k); % ��ǰ�ر�����
            z = z_all(:, k);          % ��ǰ�۲�ֵ

            % ����ر���δ��ʼ��
            if state(3+2*idx-1) == 0 && state(3+2*idx) == 0
                state(3+2*idx-1:3+2*idx) = [state(1) + z(1)*cos(state(3)+z(2));
                                             state(2) + z(1)*sin(state(3)+z(2))];
            end

            % ����Ԥ�ڹ۲�ֵ
            dx = state(3+2*idx-1) - state(1);
            dy = state(3+2*idx) - state(2);
            q = dx^2 + dy^2;
            z_pred = [sqrt(q); atan2(dy, dx) - state(3)];

            % �����ſɱȾ��� H
            H = zeros(2, length(state));
            H(:, 1:3) = [-dx/sqrt(q), -dy/sqrt(q), 0;
                         dy/q,        -dx/q,      -1];
            H(:, 3+2*idx-1:3+2*idx) = [dx/sqrt(q), dy/sqrt(q);
                                       -dy/q,      dx/q];

            % �ۻ������ſɱȾ���͹۲�в�
            H_all = [H_all; H];
            z_pred_all = [z_pred_all; z_pred];
            z_all_flat = [z_all_flat; z];
        end

        % ���㿨��������
        S = H_all * P * H_all' + kron(eye(size(measurements, 1)), R);
        K = P * H_all' / S;

        % ����۲�в�Ƕȹ�һ��
        innovation = z_all_flat - z_pred_all;
        innovation(2:2:end) = wrapToPi(innovation(2:2:end)); % �Ƕȹ�һ��

        % ����״̬��Э�������
        state = state + K * innovation;
        P = (eye(size(P)) - K * H_all) * P;
    end
end
