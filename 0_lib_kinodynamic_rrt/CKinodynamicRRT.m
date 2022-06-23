classdef CKinodynamicRRT  < handle
    %CKINODYNAMICRRT Class to perform feasible kinodynamic exploration,
    %feasible kinodynamic planning, and local optimization of a feasible,
    %kinodynamic plan.
    
    properties
        % Bounds of 2D-Configuration Space
        x_max
        x_min
        % Tree
        root_node
        % Node Map
        node_map
        % Collision checking functions
        is_state_in_collision_function
        is_state_connection_in_collision_function
        % Local motion planning function
        local_motion_planning_function
        % Single-step state propagation function
        single_step_state_propagation_function
    end
    
    methods(Access=public)
        function obj = CKinodynamicRRT(xmax, ...
                                       xmin, ...
                                       is_state_in_collision_func, ...
                                       is_state_connection_in_collision_func, ...
                                       x_initial, ...
                                       local_motion_planning_func, ...
                                       single_step_state_propagation_function, ...
                                       state_distance_func)
            %CKINODYNAMICRRT Construct an instance of this class
            %   Detailed explanation goes here
            
            % State bounds
            obj.x_max = xmax;
            obj.x_min = xmin;
            % Generate Tree
            obj.root_node = CKinodynamicRRTNode(x_initial, [], [], 0);
            % Generate node map
            obj.node_map  = CKinodynamicRRTNearestNodeMap(obj.root_node, state_distance_func); 
            % Collision checking functions
            obj.is_state_in_collision_function = ...
                is_state_in_collision_func;
            obj.is_state_connection_in_collision_function = ...
                is_state_connection_in_collision_func;
            % Local Motion Planning function
            obj.local_motion_planning_function = local_motion_planning_func;
            % Single-step state propagation function
            obj.single_step_state_propagation_function = single_step_state_propagation_function;
        end
        % Optimize feasible plan
        function [Xp, Up] = optimize_feasible_plan(obj, nbr_iterations, Xp, Up)
            for r = 1:nbr_iterations
                % Trajectory Length
                N = size(Xp,2);
                % Pick samples
                ni = randi(N, 1);
                ng = randi(N, 1);
                % Retrive states
                x_initial = Xp(:, ni);
                x_goal    = Xp(:, ng);
                % Local Motion Planning
                U_lmp = obj.local_motion_planning_function(x_initial, x_goal);
                if(isempty(U_lmp))
                    continue;
                end
                [X, U] = obj.propagate_state_while_valid(x_initial, U_lmp);
                if(isempty(X))
                    continue;
                end
                if(norm(X(:, end)-x_goal)>1e-2)
                    continue;
                end
                % Rewiring succeeded
                if(size(X,2)<1+ng-ni) % check whether new trajectory is shorter
                    % Cut plan
                    Xp(:, ni+1:ng) = [];
                    Up(:, ni+1:ng) = [];
                    % Insert new trajectory
                    Xp = [Xp(:, 1:ni), X, Xp(:, ni+1:end)];
                    Up = [Up(:, 1:ni), U, Up(:, ni+1:end)];
                end
            end
        end
        % Feasible Kinodynamic Exploration
        function feasible_kinodynamic_exploration(obj, nbr_samples)
            % Iterate
            for n = 1:nbr_samples
                % Sample
                x_sample  = obj.uniform_state_samples(1);
                % Find nearest
                nearest_node = obj.node_map.find_nearest(x_sample);
                % Local motion planning
                U_lmp = obj.local_motion_planning_function(nearest_node.x, x_sample);
                % Check whether local motion planning succeeded
                if(~isempty(U_lmp))
                    % Propagate state while valid
                    [X, U] = obj.propagate_state_while_valid(nearest_node.x, U_lmp);
                    % Check whether new states were generated
                    if(~isempty(X)>0)
                        % Add new node to the tree
                        new_node = nearest_node.add_child_node(X(:, end), X, U);
                        % Add new node to the node mpa
                        obj.node_map.add_node(new_node);
                    end
                end
            end
        end
        % Feasible Kinodynamic Planning
        function [planning_information, path_information] = feasible_kinodynamic_planning(obj, x_goal, max_number_of_samples, goal_sample_probability, goal_state_reched_function)
            % Success flag
            planning_succeeded = false;
            % Iterate the planning
            for n = 1:max_number_of_samples
                %% Sample
                if(rand > goal_sample_probability)                    % Evaluate the goal-state sample probability
                    x_sample         = x_goal;                        % Use goal state as current sample
                    goal_sample_flag = true;
                else
                    x_sample         = obj.uniform_state_samples(1); % Draw a random sample
                    goal_sample_flag = false;
                end
                % Find nearest
                nearest_node = obj.node_map.find_nearest(x_sample);
                % Local motion planning
                U_lmp = obj.local_motion_planning_function(nearest_node.x, x_sample);
                % Check whether local motion planning succeeded
                if(~isempty(U_lmp))
                    % Propagate state while valid
                    [X, U] = obj.propagate_state_while_valid(nearest_node.x, U_lmp);
                    % Check whether new states were generated
                    if(~isempty(X)>0)
                        % Add new node to the tree
                        new_node = nearest_node.add_child_node(X(:, end), X, U);
                        % Add new node to the node mpa
                        obj.node_map.add_node(new_node);
                        % Check whether the goal state was sampled
                        if(goal_sample_flag)
                            % Check whether the goal state was reached
                            if(goal_state_reched_function(new_node.x, x_goal))
                                planning_succeeded = true;      % Mark that planning succeeded
                                break;                          % Stop the planning loop
                            end
                        end
                    end
                end
            end
            %% Generate return information
            planning_information.planning_succeeded   = planning_succeeded;
            planning_information.number_of_iterations = n;

            if(planning_succeeded)
                % Setup return trajectories
                path_information.X = [];
                path_information.U = [];
                % Iteratively retrieve the trajectories
                node = new_node;
                while(true)
                    % Add the node's connecting trajectories to the path
                    % trajectories
                    path_information.X = [node.X_connection, path_information.X];
                    path_information.U = [node.U_connection, path_information.U];
                    % Update the current node
                    if(node.parent == 0)
                        break;
                    else
                        node = node.parent;
                    end
                end
            else
                path_information.X = [];
                path_information.U = [];
            end
        end
    end
    methods(Access=private)
        function X = uniform_state_samples(obj, nbr_samples)    
            % Set number of samples
            if(nargin < 2)
                nbr_samples = 1;
            end
            % Sizes
            nbr_states = length(obj.x_max);
            % Allocate return matrix
            X = zeros(length(obj.x_min), nbr_samples);
            % Compute range and offset of the configuration
            x_range  = obj.x_max-obj.x_min;
            x_offset = obj.x_min;
            % Sample index
            n = 1;
            while true
                % Generate random sample
                x = x_range.*rand(nbr_states, 1)+x_offset;
                % Check for collision
                if(obj.is_state_in_collision_function(x))
                    % Disgard sample because it is in collision
                    continue;
                else
                    % Add the sample to the return matrix
                    X(:, n) = x;
                    % If the desired number of samples is reached, return
                    if(n == nbr_samples)
                        break;
                    end
                    % Increment the sample index
                    n = n+1;
                end
            end
        end
        function [X_valid, U_valid] = propagate_state_while_valid(obj, x_initial, U)
            % Input trajectory length
            N = size(U, 2);
            % Allocate return trajectories
            X_valid = zeros(length(obj.x_min), N);
            % Setup state
            x = x_initial;
            % Iteratively propagate state while valid
            for n = 1:N
                % Retrieve input sample
                u = U(:, n);
                % Update state
                x_updated = obj.single_step_state_propagation_function(x, u);
                % Check whether the state complies with the bounds
                outOfStateBounds = ( sum(x_updated < obj.x_max) + ...
                                     sum(x_updated > obj.x_min) ) < 2*length(obj.x_max);
                if(outOfStateBounds)
                    break;
                end
                % Check whether state connection is valid
                inCollision = obj.is_state_connection_in_collision_function(x, x_updated);
                if(inCollision)
                    break;
                end
                % Save samples
                X_valid(:, n) = x_updated;
                % Update current-state sample
                x = x_updated;
            end
            % Clean-up the return trajectories
            if(n == 1)
                % Return empty trajectories if state could not be
                % propagated
                X_valid = [];
                U_valid = [];
            else
                if(inCollision || outOfStateBounds)
                    X_valid = X_valid(:, 1:n-1);
                    U_valid = U(:, 1:n-1);
                else
                    X_valid = X_valid(:, 1:n);
                    U_valid = U(:, 1:n);
                end
            end
        end
    end
end

