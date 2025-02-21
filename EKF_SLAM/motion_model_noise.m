% �˶�ģ��
function state = motion_model_noise(state, u, dt, Q)
    noise = chol(Q)' * randn(2,1);
    v = u(1) + noise(1);   % �ٶ�
    w = u(2) + noise(2);   % ת����ٶ� 
    theta = state(3);
%     dx = v * cos(theta) * dt;
%     dy = v * sin(theta) * dt;
%     dtheta = w * dt;
    
    dx = (v/w)*(sin(theta+w*dt)-sin(theta));
    dy = (v/w)*(cos(theta)-cos(theta+w*dt));
    dtheta = w * dt;
    
    state = state + [dx; dy; dtheta];
end
