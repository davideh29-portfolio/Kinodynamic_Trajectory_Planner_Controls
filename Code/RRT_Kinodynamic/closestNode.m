function [closest_node] = closestNode(T, x_test)
% Find the closest node in the tree to a testing node

% Check if no children
if isempty(T.children)
    closest_node = T;
    return
end

% Create queue
queue = Queue();
queue = queue.push(T);

% Loop through nodes in queue and find closest
rho = zeros(1, length(queue.nodes));
for i = 1:length(queue.nodes)
    rho(i) = get_rho(x_test, queue.nodes{i}.x);
end
[~, I] = min(rho);
closest_node = queue.nodes{I(1)};


end

