function [] = sim_path(path, spheres)
% Simulate path specified by joint coordinates in input "path"

% Start arm simulation
lynxStart();

hold on;
[x, y, z] = sphere;
% Plot spheres in "spheres" object
for i = 1:length(spheres.positions)
    surf((x*spheres.r{i} + spheres.positions{i}(1)),...
        (y*spheres.r{i} + spheres.positions{i}(2)),...
        (z*spheres.r{i} + spheres.positions{i}(3)));
end

% For visualization of end effector and joint paths
joint_3_path = zeros(size(path, 2), 3);
joint_4_path = zeros(size(path, 2), 3);

% Simulate path
for i = 1:size(path, 1)
    pause(.1);
    lynxServoSim([path(i, 1:3) 0 0 0]);
    pos = updateQ([path(i, 1:3)]);
    joint_3_path(i, :) = pos(3, :);
    joint_4_path(i, :) = pos(4, :);
end
plot3(joint_3_path(:, 1), joint_3_path(:, 2), joint_3_path(:, 3), 'LineWidth', 2);
plot3(joint_4_path(:, 1), joint_4_path(:, 2), joint_4_path(:, 3), 'LineWidth', 2);
end

