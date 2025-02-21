% FEKF���²���
function [state, P ,first_obs] = fekf_update(state, P, measurements, R, first_obs)
    % first_obsΪһ��2*n+3��2�еľ��󣬵�һ�б����״ι۲⵽landmark�����꣬�ڶ��б����Ӧ�״ι۲⵽landmarkʱС��������
    for k = 1:size(measurements,1)
        idx = measurements(k,1);
        z = measurements(k,2:3)';
        % �����µر�ʱ
        if state(3+2*idx-1) == 0 && state(3+2*idx) == 0
           state(3+2*idx-1:3+2*idx) = [state(1) + z(1)*cos(state(3)+z(2));
                                       state(2) + z(1)*sin(state(3)+z(2))];                            
           first_obs(3+2*idx-1:3+2*idx,1) = state(3+2*idx-1:3+2*idx);
           first_obs(3+2*idx-1:3+2*idx,2) = [state(1);state(2)];                         
        end     
%% ����Ԥ�ڲ���ֵ
        dx = state(3+2*idx-1) - state(1);
        dy = state(3+2*idx) - state(2);
        q = dx^2 + dy^2;
        z_pred = [sqrt(q);atan2(dy, dx) - state(3)];

%% ����H����
%         dx = first_obs(3+2*idx-1,1) - first_obs(3+2*idx-1,2);
%         dy = first_obs(3+2*idx,1) - first_obs(3+2*idx,2);
%         q = dx^2 + dy^2;

        dx = first_obs(3+2*idx-1,1) - state(1);
        dy = first_obs(3+2*idx,1) - state(2);
        q = dx^2 + dy^2;
        
        % �ſɱȾ���
        H = zeros(2, length(state));
        H(:,1:3) = [-dx/sqrt(q), -dy/sqrt(q), 0;
                    dy/q,        -dx/q,      -1];
        H(:,3+2*idx-1:3+2*idx) = [dx/sqrt(q), dy/sqrt(q);
                                  -dy/q,      dx/q];                          
    %% ���¼���                              
        % ����������
        K = P*H'/(H*P*H' + R);
        % ״̬����
        innovation = z - z_pred;
        innovation(2) = wrapToPi(innovation(2)); % �Ƕȹ�һ��
        state = state + K*innovation;
        P = (eye(size(P)) - K*H) * P;
    end
end

