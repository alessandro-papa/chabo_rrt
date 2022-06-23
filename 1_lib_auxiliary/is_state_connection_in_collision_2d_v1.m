function inCollision = is_state_connection_in_collision_2d_v1(x1, x2, x2y_function, obstacle_cell)
    % Retrieve output samples
    y1 = x2y_function(x1);
    y2 = x2y_function(x2);
    % Check whether the output samples are in collision
    if(is_state_in_collision_2d_v1(x1, x2y_function, obstacle_cell) || ...
       is_state_in_collision_2d_v1(x2, x2y_function, obstacle_cell))
        inCollision = true;
        return;
    end
    % Check whether the line is in collision
    inCollision = false;
    % Line coordinates
    yx = [y1(1); y2(1)];
    yy = [y1(2); y2(2)];
    % Iteratively check all obstacles for collision
    for n = 1:length(obstacle_cell)
        % Retireve the obstacle
        obstacle = obstacle_cell{n,1};
        % Check for a collision
        xi = polyxpoly(yx, yy, obstacle.Vertices(:,1), obstacle.Vertices(:,2));
        if(isempty(xi) == false)
            inCollision = true;
            break;
        end
    end
end

