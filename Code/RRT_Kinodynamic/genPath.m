function path = genPath(T_start, T_end, n)
% Generates path from Tstart to Tend connected by an intermediate node n

% Initialize variables
path = {n};
n_1 = n.parent{1};
n_2 = n.parent{2};
path_1 = {n_1};
path_2 = {n_2};
path_1_done = isempty(n_1.parent);
path_2_done = isempty(n_2.parent);
i = 2;

% Loop until first and last nodes are reached
while ~path_1_done || ~path_2_done
    % Update paths if not complete
    if ~path_1_done
        path_1{i} = n_1.parent{1};
        % Check if they equal T_start or T_end
        if path_1{i} == T_start || path_1{i} == T_end
            path_1_done = true(1);
        else
            n_1 = n_1.parent{1};
        end
    end
    if ~path_2_done
        path_2{i} = n_2.parent{1};
        if path_2{i} == T_start || path_2{i} == T_end
            path_2_done = true(1);
        else
            n_2 = n_2.parent{1};
        end
    end
    i = i + 1;
end

% Combine paths into final output
if path_1{end} == T_start
    path = [flip(path_1), path, path_2];
else
    path = [flip(path_2), path, path_1];
end

end

