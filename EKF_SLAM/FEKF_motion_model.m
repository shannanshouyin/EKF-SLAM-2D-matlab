% 运动模型
function state = FEKF_motion_model(state, state_pre, u, dt)
    v = u(1);   % 速度
    w = u(2);   % 转向角速度
    theta = state_pre(3);
    
    dx = v * cos(theta) * dt;
    dy = v * sin(theta) * dt;
    dtheta = w * dt;
    
    state = state + [dx; dy; dtheta];
end
