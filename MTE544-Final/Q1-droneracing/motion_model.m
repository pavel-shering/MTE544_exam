%% motion model
%
% Input: 
% xprev = [x, y, theta]' 
% u = [vx vy theta]';
% vx     - speed x
% vy     - speed y 
% theta  - angle
%        
% Output:
% xcur - new robot state [x, y, theta]'

function xcur = motion_model(xprev, u, dt)
    dt_b = dt*[u];
    dt_b(3) = u(3)
    R = rot(-xprev(3),3);
    
    xcur = xprev + R*dt_b;
%     xcur(3) = u(3);
    u
end 
