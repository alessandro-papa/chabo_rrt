classdef CKinodynamicRRTNode < handle
    %CKINODYNAMICRRTNODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x
        X_connection
        U_connection
        parent
        child_cell
    end
    
    methods
        function obj = CKinodynamicRRTNode(x, X_connection, U_connection, parent)
            %CKINODYNAMICRRTNODE Construct an instance of this class
            %   Detailed explanation goes here
            % State of the node
            obj.x = x;
            % Input trajectory connecting parent state to this node's state
            obj.U_connection = U_connection;
            % State trajectory connecting parent state to this node's state
            obj.X_connection = X_connection;
            % Parent node
            obj.parent = parent;
            % Cell holding all child nodes
            obj.child_cell = cell(0,0);
        end
        function child_node = add_child_node(obj, x, X_connection, U_connection)
            % Create child node
            child_node = CKinodynamicRRTNode(x, X_connection, U_connection, obj);
            % Add new node to list of child nodes
            obj.child_cell{end+1,1} = child_node;
        end
        function draw(obj, x2y_function)
            % Compute output samples
            y_parent = x2y_function(obj.x);
            % Marker for output sample
            plot(y_parent(2), y_parent(1), '.', 'MarkerSize', 14,'LineWidth', 1.2, 'Color', '#377eb8'); hold on;
            for n = 1:length(obj.child_cell)
                % Fetch child node
                child_node = obj.child_cell{n,1};
                % Compute the output trajectory
                Y_connection = x2y_function([obj.x, child_node.X_connection]);
                % Draw line to child node
                line(Y_connection(2, :), Y_connection(1,:), 'LineWidth', 1.2, 'Color', '#377eb8');
                % Call child node's draw
                child_node.draw(x2y_function);
            end
        end
    end
end

