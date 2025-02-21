% 运动模型
% Xk+1 = f(Xk,Uk)
% X:(x,y,theta) U:(v,w)
function state = motion_model(state, u, dt)
    v = u(1);   % 速度
    w = u(2);   % 转向角速度
    theta = state(3);
    
%     dx = v * cos(theta) * dt;
%     dy = v * sin(theta) * dt;
%     dtheta = w * dt;

    dx = (v/w)*(sin(theta+w*dt)-sin(theta));
    dy = (v/w)*(cos(theta)-cos(theta+w*dt));
    dtheta = w * dt;

    state = state + [dx; dy; dtheta];
end


