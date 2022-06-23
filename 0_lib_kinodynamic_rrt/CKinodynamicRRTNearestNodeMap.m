classdef CKinodynamicRRTNearestNodeMap < handle
    %CKINODYNAMICRRTNEARESTNODEMAP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        node_cell
        X
        state_distance_function
    end
    
    methods
        function obj = CKinodynamicRRTNearestNodeMap(root_node, state_distance_func)
            %CKINODYNAMICRRTNEARESTNODEMAP Construct an instance of this class
            %   Detailed explanation goes here
            % Setup the cell containing all nodes
            obj.node_cell = {root_node};
            % Setup the array containing all states
            obj.X         = root_node.x';
            state_distance_function = state_distance_func;
        end
        function add_node(obj, node)
            obj.node_cell{end+1,1} = node;
            obj.X(end+1,:)         = node.x';
        end
        function nearest_node = find_nearest(obj, x)
            % Find nearest index
            nearest_idx  = knnsearch(obj.X, x', 'Distance', 'euclidean');
            % Return nearest node
            nearest_node = obj.node_cell{nearest_idx,1};
        end
        function nearest_node_cell = find_K_nearest_nodes(obj, x, K)
            % Find nearest indices
%             nearest_indices = knnsearch(obj.X, x', 'K', K, 'Distance', 'euclidean');
            nearest_indices = knnsearch(obj.X, x', 'K', K, 'Distance', state_distance_function(x,Y));
            % Return nearest nodes
            nearest_node_cell = obj.node_cell(nearest_indices, 1);
        end
    end
end

