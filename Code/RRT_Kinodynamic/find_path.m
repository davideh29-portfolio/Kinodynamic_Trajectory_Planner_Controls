function [ x, pathPossible ] = find_path( T_start, T_end, spheres)
n_torques = 100; % Number of torques to test for each step
t_lim = .001; % Max torque that can be applied in either direction for all joints
v_lim = .3; % Max angular velocity for each joint
dt = .01; % Fixed time step for change in state due to torque
epsilon = 3; % Rho value accepted as having reached a state
inBoth = false(1);
pathPossible = ~spheres.checkConfig(T_start.x(1:3)) && ~spheres.checkConfig(T_end.x(1:3));
x = [];
best_rho1 = Inf;
best_rho2 = Inf;
if ~pathPossible
    return
end

% Loop until path is found
while ~inBoth
    % Create new testing state
    x_test = rand(1, 6).*[2.8 2.6 3.5 v_lim*2*ones(1, 3)]...
        - [1.4 1.2 1.8 v_lim*ones(1, 3)];
    n_test = Node(x_test);
    inBoth = true(1);
    % Find closest point in T_start, x_a using rho metric
    n_a = closestNode(T_start, x_test);
    % Loop until collision or point reached
    prev_best_rho = Inf;
    while true()
        % Loop n_torques times
        rt = zeros(n_torques, 3);
        rho = zeros(n_torques, 1);
        x_new = zeros(n_torques, 6);
        for i = 1:n_torques
            % Random torque
            rt(i, :) = rand(1, 3).*t_lim*2.*ones(1, 3) - t_lim*ones(1, 3);
            % Evaluate torque with rho metric
            [rho(i), x_new(i, :)] = torque_eval(n_a, x_test, rt(i, :), dt);
        end
        % Create node for best torque
        [r, I] = min(rho);
        node_new = Node(x_new(I, :));
        % Check for reaching target state or collision
        if r > prev_best_rho
            inBoth = false();
            break;
        else
            prev_best_rho = r;
            if prev_best_rho < best_rho1
                best_rho1 = prev_best_rho;
            end
        end
        if spheres.checkConfig(x_new(i, 1:3))
            % End loop
            inBoth = false();
            break;
        elseif r < epsilon % target is reached
            % Add node to tree
            r1 = r
            n_a.children{length(n_a.children) + 1} = n_test;
            n_test.parent{length(n_test.parent) + 1} = n_a;
            % End loop
            break;
        else
            % Add node to tree
            n_a.children{length(n_a.children) + 1} = node_new;
            node_new.parent{length(node_new.parent) + 1} = n_a;
            n_a = node_new;
        end
    end
    % Find closest point in T_end, n_b using rho metric
    n_b = closestNode(T_end, x_test);
    % Loop until collision or point reached
    prev_best_rho = Inf;
    while true()
        % Loop n_torques times
        rt = zeros(n_torques, 3);
        rho = zeros(n_torques, 1);
        x_new = zeros(n_torques, 6);
        for i = 1:n_torques
            % Random torque
            rt(i, :) = rand(1, 3).*t_lim*2.*ones(1, 3) - t_lim*ones(1, 3);
            % Evaluate torque with rho metric
            [rho(i), x_new(i, :)] = torque_eval(n_b, x_test, rt(i, :), dt);
        end
        % Create node for best torque
        [r, I] = min(rho);
        node_new = Node(x_new(I, :));
        if r > prev_best_rho
            inBoth = false();
            break;
        else
            prev_best_rho = r;
            if prev_best_rho < best_rho2
                best_rho2 = prev_best_rho;
            end
        end
        % Check for reaching target state or collision
        if spheres.checkConfig(x_new(i, 1:3))
            % End loop
            inBoth = false();
            break;
        elseif r < epsilon % target is reached
            r2 = r
            % Add node to tree
            n_b.children{length(n_b.children) + 1} = n_test;
            n_test.parent{length(n_test.parent) + 1} = n_b;
            % End loop
            break;
        else
            % Add node to tree
            n_b.children{length(n_b.children) + 1} = node_new;
            node_new.parent{length(node_new.parent) + 1} = n_b;
            n_b = node_new;
        end
    end
    % Check if node is in both trees - if so, generate path and output
    if inBoth
        path = genPath(T_start, T_end, n_test);
        disp(strcat('Number of parents: ', int2str(length(n_test.parent))));
    end
end
% Output final path in state-space
x = zeros(size(path, 2), 6);
for c = 1:size(path, 2)
    x(c, :) = path{c}.x;
end

end

