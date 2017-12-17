%% p - heading traj 
% Input: 
% target - [x y] of the target point
% x - [x y theta] of the current robot pos
% prev_u  - [vx vy theta] previous inputs
%        
% Output:
% u - [vx vy theta] current input

function [u] = traj(target, x, prev_u, r_m, phi_m ,not_these)
    error_threshold = 0.1;
    theta_ref = atan2(target(2)-x(2), target(1)-x(1))
    theta_error = theta_ref - x(3)
    
    closest = abs(theta_error - phi_m(round(length(phi_m/2))));
    heading_index = round(length(phi_m/2));
    for i = 1:length(phi_m)

        closest_cur = abs(theta_error - phi_m(i));
        if (closest_cur < closest) && ~(not_these(i))
            closest = closest_cur;
            heading_index = i;
        end

    end
%     heading_index
    u = prev_u'
    u(3) = phi_m(heading_index)
end
