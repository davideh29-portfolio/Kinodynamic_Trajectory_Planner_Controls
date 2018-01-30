clear;
%% Set hardware flag and control constants
hardware_flag = false;

% Controller constants
k_i = -10000.0;
k_p = -800000.0;
k_d = -1000;

% Simulation conditions
noise_magnitude_measured = .01; % Magnitude of gaussian noise for sensor
noise_magnitude_sim = 2000;     % Magnitude of gaussian noise for torque
f = 10;                         % Sin wave frequency
w = .6;                         % Velocity filter weight

%% Set initial conditions
num_steps = 100000;                     % Amount of steps to run simulation
cum_time = 0;                           % Total time simulation has run
dt = 0.001;                             % Time step computed by tic toc
area_under_curve = 0;                   % Integral for PID control
e_theta = 0;                            % Error in theta in current loop
e_theta_prev = 0;                       % Error in theta in last loop
sim_time = 100;
actual_t = linspace(0, sim_time, num_steps); % Actual time points for plotting
theta_desired = zeros(1, 3);            % Desired theta

% Initialize measured position and velocity variables
theta_prev = sin(f*0)*ones(1, 3);
theta_dot_prev = f*cos(f*0)*ones(1, 3);
theta = sin(f*0)*ones(1, 3);
theta_dot = f*cos(f*0)*ones(1, 3);

% Initialize simulated (actual) position, velocity and acceleration variables
theta_sim = sin(f*0)*ones(1, 3);
theta_dot_sim = f*cos(f*0)*ones(1, 3);
theta_dot_dot_sim = -f^2*sin(f*0)*ones(1, 3);

% Plotting variables
theta_plot_sim = zeros(num_steps, 3);
theta_dot_plot_sim = zeros(num_steps, 3);
theta_dot_dot_plot_sim = zeros(num_steps, 3);

theta_plot_measured = zeros(num_steps, 3);
theta_dot_plot_measured = zeros(num_steps, 3);
theta_dot_dot_plot_measured = zeros(num_steps, 3); 

%% Main loop
tic;

i = 1;
while cum_time <= sim_time
    i = i + 1;
    
    % Calculate time step
    if hardware_flag
        dt = toc;
        tic;
        actual_t(i) = actual_t(i-1) + dt;
    else
        actual_t(i) = actual_t(i-1) + dt;
    end
    
    % Calculate desired position and velocity values (for planned
    % trajectory)
    cum_time = cum_time + dt;
    theta_desired = ones(1, 3) * sin(f*cum_time);
    theta_dot_desired = ones(1, 3) * f*cos(f*cum_time);
    theta_dot_dot_desired = ones(1, 3) * -f^2*sin(f*cum_time);
    
    % Update plot of simulated values
        theta_plot_sim(i-1, :) = theta_sim;
        theta_dot_plot_sim(i-1, :) = theta_dot_sim;
        theta_dot_dot_plot_sim(i-1, :) = theta_dot_dot_sim;
        theta_plot_measured(i-1, :) = theta;
        theta_dot_plot_measured(i-1, :) = theta_dot;
    
    % Read position values from hardware flag
    if hardware_flag
        % Based on simulated acceleration, simulate next position and velocity
        theta_sim = lynxGetAngles;
        % Simulate noisy measurement of simulated/actual theta
        theta_prev = theta;
        theta = theta_sim + noise_magnitude_measured * randn(1, 3);
        % Compute velocity estimate (measured)
        change_theta = theta - theta_prev;
        theta_dot = w*(change_theta / dt) + (1-w)*theta_dot_prev;        
    else
        % Based on simulated acceleration, simulate next position and velocity
        theta_dot_sim = theta_dot_sim + theta_dot_dot_sim*dt;
        theta_sim = theta_sim + theta_dot_sim*dt;
        % Simulate noisy measurement of simulated/actual theta
        theta_prev = theta;
        theta = theta_sim + noise_magnitude_measured * randn(1, 3);
        % Compute velocity estimate (measured)
        change_theta = theta - theta_prev;
        theta_dot = w*(change_theta / dt) + (1-w)*theta_dot_prev;
    end
    
    % Calculate the error between measured theta and desired theta
    e_theta_prev = e_theta;
    e_theta = theta - theta_desired;
    
    % Calculate error measured theta_dot and desired theta_dot
    e_theta_dot = theta_dot - theta_dot_desired;
    
    % Calculate integral of angular position approx
    % Use trapezoidal approximation
    error_avg = (e_theta + e_theta_prev)./2;
    area_under_curve = area_under_curve + error_avg * dt;
    
    % Apply controller to compute torques
    Tau_control = k_p * e_theta + k_d * e_theta_dot + k_i * area_under_curve;
    
    % Compute Tau for desired acceleration of planned trajectory
    [M, N, C] = computeMNC(theta_desired, theta_dot_desired);
    Tau_desired = (M*theta_dot_dot_desired' + C*theta_dot_desired' + N)';
    
    % Simulate error in torque applied
    Tau_error = noise_magnitude_sim * randn(1, 3);
    
    % Compute resulting acceleration for Tau + Tau_desired in simulation
    theta_dot_dot_sim = (M\((Tau_control + Tau_desired + Tau_error)'...
        - C*theta_dot_sim' - N))';
    
    if hardware_flag
        % Send torques to lynx
        if figClosed % quit by closing the figure
            lynxDCTorquePhysical(0,0,0,0,0,0);
            return;
        else
            currents = torquesToCurrents(Tau_control + Tau_desired + Tau_error);
            lynxDCTorquePhysical(currents(1),currents(2),currents(3),0,0,0);
        end
    end
    
end

%% Plot optimal path
t = linspace(0, sim_time, 100001);
theta_optimal = sin(f*t);
theta_dot_optimal = f*cos(f*t);
theta_dot_dot_optimal = -f^2*sin(f*t);

% Plot measured path
figure
plot(actual_t(1:i-1), theta_plot_measured(1:i-1, 1), 'Color', [.85 .325 .098]);
hold on;
xlim([0 10]);
plot(t, theta_optimal, 'Color', [0 .4470 .741], 'LineWidth', 4);
plot(actual_t(1:i-1), theta_plot_sim(1:i-1, 1), 'Color', [.929 .694 .125], 'LineWidth', 2);
legend('Measured theta', 'Desired theta', 'Actual theta', 'Location', 'northwest');

% Plot measured path
figure
plot(actual_t(1:i-1), theta_dot_plot_measured(1:i-1, 1), 'Color', [.85 .325 .098]);
hold on;
xlim([10 20]);
plot(t, theta_dot_optimal, 'Color', [0 .4470 .741], 'LineWidth', 4);
plot(actual_t(1:i-1), theta_dot_plot_sim(1:i-1, 1), 'Color', [.929 .694 .125], 'LineWidth', 2);
legend('Measured vel', 'Desired vel', 'Actual vel', 'Location', 'northwest');