% EKFÔ¤²â²½Öè
function [state, P] = ekf_predict(state, P, u, dt, Q)
    v = u(1);
    w = u(2); 
    theta = state(3);
    
    % xµÄÑÅ¿É±È¾ØÕó
    F = eye(size(P));
%     F(1,3) = -v*sin(theta)*dt;
%     F(2,3) = v*cos(theta)*dt;
    F(1,3) = (v/w)*(cos(theta+w*dt)-cos(theta));
    F(2,3) = (v/w)*(sin(theta+w*dt)-cos(theta));
    
    % uµÄÑÅ¿É±È¾ØÕó
%     G = [cos(theta)*dt  0;
%          sin(theta)*dt  0;
%          0              dt];
    G = [(1/w)*(sin(theta+w*dt)-sin(theta))     -(v/(w^2))*(sin(theta+w*dt)-sin(theta))+(v/w)*dt*cos(theta+w*dt);
         (1/w)*(cos(theta)-cos(theta+w*dt))     -(v/(w^2))*(cos(theta)-cos(theta+w*dt))+(v/w)*dt*sin(theta+w*dt);
          0              dt];
    
    % ×´Ì¬Ô¤²â
    state(1:3) = motion_model(state(1:3), u, dt);
    
    % Ð­·½²îÔ¤²â
    P(1:3,1:3) = F(1:3,1:3)*P(1:3,1:3)*F(1:3,1:3)' + G * Q * G';
    P(1:3,4:end) = F(1:3,1:3)*P(1:3,4:end);
    P(4:end,1:3) = P(4:end,1:3)*F(1:3,1:3)';
    P(4:end,1:3) = P(1:3,4:end)';
end
