classdef Node < handle
    
    properties
        children = {}
        parent = {}
        tree = ''
        x = zeros(1, 6);
    end
    
    methods
        % Constructor
        function obj = Node(x)
            obj.x = x;
        end        
    end
    
end

