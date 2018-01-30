classdef Spheres < handle
    % Spheres object containing a position in 3D space and radius
    
    properties
        r = {};
        positions = {};
        n_step = 100;
        config = false([100, 100, 100]);
    end
    
    methods(Static)
        
        % Check if angle between sphere and link is obtuse
        function obtuse = isObtuse(p1, p2, sphere_center)
            d = zeros(1, 3);
            d(1) = norm(p1-p2);
            d(2) = norm(p2-sphere_center);
            d(3) = norm(p1-sphere_center);
            [m, ind] = max(d);
            if (ind == 1)
                obtuse = 0;
            else
                obtuse = d(ind)^2 > sum((d(d ~= m)).^2);
            end
        end
        
        % Check if sphere is far enough from link
        function d = pointToLine(v1, v2, p1)
            a = v1-v2;
            b = p1-v2;
            d = norm(cross(a, b)/norm(a));
        end
        
    end
    
    methods
        
        function collide = isCollide(obj, j1, j2, i, robot_pos, l_width)
            % Boolean for collision
            collide = false();
            % Check if obtuse
            if (Spheres.isObtuse(robot_pos(j1, 1:3), robot_pos(j2, 1:3), obj.positions{i}))
                % Check if distance from link to sphere positions is
                % greater than width + radius
                if (norm(robot_pos(j1, :)-obj.positions{i}) > (l_width(j1) + obj.r{i}))
                    
                elseif (norm(robot_pos(j2, :)-obj.positions{i}) > (l_width(j1) + obj.r{i}))
                    
                else
                    collide = true();
                end
            else
                % If not obtuse, check distance from line to sphere center
                dist = Spheres.pointToLine(robot_pos(j1, 1:3), ...
                    robot_pos(j2, 1:3), obj.positions{i});
                if (dist < (l_width(j1) + obj.r{i}))
                    collide = true();
                end
            end
        end
        
        function convertToConfig(obj)
            l_width(2) = 5 * 10;          % in mm
            l_width(3) = 2 * 10;
            
            av = linspace(-1.4, 1.4, obj.n_step);
            bv = linspace(-1.2, 1.4, obj.n_step);
            cv = linspace(-1.8, 1.7, obj.n_step);
            
            % Loop through joint variables and generate 4D config matrix
            for a = 1:length(av)
                for b = 1:length(bv)
                    for c = 1:length(cv)
                        for i = 1:length(obj.r)
                            [robot_pos, ~] = updateQ([av(a), bv(b), cv(c)]);
                            % line between joint 2 and joint 3
                            if (isCollide(obj, 2, 3, i, robot_pos, l_width))
                                obj.config(a, b, c) = 1;
                                continue;
                            elseif (isCollide(obj, 3, 4, i, robot_pos, l_width))
                                obj.config(a, b, c) = 1;
                                continue;
                            else
                                % No collision
                            end
                        end
                    end
                end
            end
        end
        
        % Check if a given configuration collides
        function collides = checkConfig(obj, q)
            % Find index closest to configuration
            av = linspace(-1.4, 1.4, obj.n_step);
            [~, I1] = min(abs(av - q(1)));
            bv = linspace(-1.2, 1.4, obj.n_step);
            [~, I2] = min(abs(bv - q(2)));
            cv = linspace(-1.8, 1.7, obj.n_step);
            [~, I3] = min(abs(cv - q(3)));
            % Check config-space graph to check for collision
            collides = obj.config(I1, I2, I3);
        end
        
        % Constructor
        function obj = Spheres(coord, r)
            if nargin == 2
                obj.positions = coord;
                obj.r = r;
                
            else
                for i = 1:4
                    obj.positions{i} = ((rand(1, 3) .* [32, 32, 20]) - [16, 16, 0]) * 25.4;
                    obj.r{i} = rand(1) * 2 * 25.4;
                end
            end
            convertToConfig(obj)
        end
    end
    
end