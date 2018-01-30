%% Create spheres object and calculate obstacles in configuration space
clear;

tic
% 6 spheres example
spheres = Spheres({[-5*25.4, -5*25.4, 250], [150, 150, 350], ...
    [5*25.4, 0, 250], [0, -150, 300], [50, 100, 100], [-100, -200, 100]}, {60, 40, 80, 50, 60, 40});
toc

save('Spheres_15', 'spheres');

%% Test overall RRT
clear;

load('Spheres_15.mat')
load('Inputs');
% Node structure
% Convert input points into joint parameters
q_start = IK_lynx([100, 150, 200]);
q_end = IK_lynx([200, -200, 200]);

% Initializing trees
T_start = Node([q_start 0 0 0]);
T_end = Node([q_end 0 0 0]);

% Find path
tic
[path, pathPossible] = find_path(T_start, T_end, spheres);
toc

%% Simulate path
load('good_path_1')
if pathPossible
    sim_path(path, spheres);
else
    disp('Path impossible');
end
