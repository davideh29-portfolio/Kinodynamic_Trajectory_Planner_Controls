function [rho, x_new] = torque_eval(n_curr, x_test, rt, dt)

% Compute new state from applied torque
[M, N, C] = computeMNC(n_curr.x(1, 1:3)', n_curr.x(1, 4:6)');
accel = (M\(rt' - C*n_curr.x(1, 4:6)' - N))';   % Calculate accel from torque
x_new(4:6) = n_curr.x(4:6) + accel*dt;          % Calculate new velocity
x_new(1:3) = n_curr.x(1:3) + n_curr.x(4:6)*dt;  % Calculate new position

% Compute metric rho for difference in position and velocity
rho = get_rho(x_new, x_test);

end

