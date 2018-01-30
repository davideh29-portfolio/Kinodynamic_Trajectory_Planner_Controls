function [rho] = get_rho(x_new, x_target)
% Compute metric rho for difference in position and velocity
weight_pos = 5;
weight_vel = 2;
diff_pos = x_target(1:3) - x_new(1:3);
diff_vel = x_target(4:6) - x_new(4:6);
rho = weight_pos*norm(diff_pos) + weight_vel*norm(diff_vel);
end

