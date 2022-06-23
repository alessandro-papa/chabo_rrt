function [obstacle_cell_flipped] = flip_obstacle(obstacle_cell)
%FLIP_OBSTACLE Summary of this function goes here
%   Detailed explanation goes here
    obstacle_cell_flipped = cell(size(obstacle_cell));
    for i=1:size(obstacle_cell)
        obstacle_cell_flipped{i}    = obstacle_cell{i};
        tmpVertices                 = zeros(size(obstacle_cell{i}.Vertices));
        tmpVertices(:,1)            = obstacle_cell{i}.Vertices(:,2);
        tmpVertices(:,2)            = obstacle_cell{i}.Vertices(:,1);
        obstacle_cell_flipped{i}.Vertices = tmpVertices;
    end
end

