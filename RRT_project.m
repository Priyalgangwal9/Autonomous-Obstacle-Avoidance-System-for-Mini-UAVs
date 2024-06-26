% Define the map
map = [
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
        0, 1, 0, 0, 0, 1, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0
];

% Define start and goal positions
start = [1, 1];
goal = [10, 10];

% Define RRT parameters
max_iter = 1000; % Maximum number of iterations
step_size = 1.0; % Step size for extending the tree

% Run RRT algorithm
[path, tree] = rrt(map, start, goal, max_iter, step_size);

% Plot the path and tree
plot_rrt(map, path, tree);


function [path, tree] = rrt(map, start, goal, max_iter, step_size)
    % Initialize the tree with the start node
    tree = [start, 0]; % Each node contains x, y, and parent index
    
    for iter = 1:max_iter
        % Sample a random point in the map
        random_point = [randi(size(map, 1)), randi(size(map, 2))];
        
        % Find the nearest node in the tree to the random point
        nearest_node_idx = nearest_neighbor(tree(:, 1:2), random_point);
        nearest_node = tree(nearest_node_idx, :);
        
        % Extend the tree towards the random point
        new_node = steer(nearest_node(1:2), random_point, step_size);
        
        % If the new node is not in collision with obstacles
        if ~check_collision(map, nearest_node(1:2), new_node)
            % Add the new node to the tree
            tree = [tree; new_node, nearest_node_idx];
            
            % Check if the goal is reached
            if norm(new_node - goal) < step_size
                path = reconstruct_path(tree, size(tree, 1));
                return;
            end
        end
    end
    
    % If the goal is not reached within the maximum iterations
    disp('Goal not reached within maximum iterations');
    path = [];
end

function nearest_node_idx = nearest_neighbor(tree, random_point)
    % Find the index of the nearest node in the tree to the random point
    distances = vecnorm(tree - random_point, 2, 2);
    [~, nearest_node_idx] = min(distances);
end

function new_node = steer(nearest_node, random_point, step_size)
    % Move from the nearest node towards the random point by step_size
    direction = random_point - nearest_node;
    distance = norm(direction);
    if distance <= step_size
        new_node = random_point;
    else
        new_node = nearest_node + (direction / distance) * step_size;
    end
end

function in_collision = check_collision(map, nearest_node, new_node)
    % Check if the line segment between nearest_node and new_node intersects with obstacles
    x1 = nearest_node(1);
    y1 = nearest_node(2);
    x2 = new_node(1);
    y2 = new_node(2);
    
    in_collision = any(map(round(linspace(x1, x2, 100)), round(linspace(y1, y2, 100))));
end

function path = reconstruct_path(tree, goal_idx)
    % Reconstruct the path from the goal node to the start node
    path = tree(goal_idx, 1:2);
    parent_idx = tree(goal_idx, 3);
    while parent_idx ~= 0
        path = [tree(parent_idx, 1:2); path];
        parent_idx = tree(parent_idx, 3);
    end
end

function plot_rrt(map, path, tree)
    % Plot map
    imagesc(map);
    colormap(flipud(gray));
    hold on;
    
    % Plot path
    if ~isempty(path)
        plot(path(:, 2), path(:, 1), 'r', 'LineWidth', 2);
    end
    
    % Plot tree
    plot(tree(:, 2), tree(:, 1), 'bo', 'MarkerSize', 3);
    
    % Plot start and goal positions
    plot(tree(1, 2), tree(1, 1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(tree(end, 2), tree(end, 1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Set axis properties
    axis equal;
    axis([1 size(map, 2) 1 size(map, 1)]);
    grid on;
    
    hold off;
end