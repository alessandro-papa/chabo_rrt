function inCollision = is_state_in_collision_2d_v1(x, x2y_funcion, obstacle_cell)
    % Obtain output sample
    y = x2y_funcion(x);
    % Allocate return
    inCollision = false;
    % Iteratively check collision with all obstacles
    for k = 1:length(obstacle_cell)
        % Retrieve obstacle
        obstacle = obstacle_cell{k};
        % Check for collision
        if(inpolygon(y(1,1), y(2,1), obstacle.Vertices(:,1), obstacle.Vertices(:,2)))
                    inCollision = true;
                    break;
        end
    end
end

