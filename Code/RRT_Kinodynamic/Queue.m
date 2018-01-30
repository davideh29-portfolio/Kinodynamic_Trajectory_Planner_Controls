classdef Queue
    
    properties
        nodes = {}
    end
    
    methods
        
        % Add a new node to the queue
        function obj = push(obj, node)
            obj.nodes{length(obj.nodes) + 1} = node;
            for i = 1:length(node.children)
                obj = push(obj, node.children{i});
            end
        end
        
        % Pop a node from the queue
        function node_pop = pop(node)
            node_pop = obj.nodes{size(obj.nodes)};
            obj.nodes{size(obj.nodes)} = [];
        end
        
    end
    
end