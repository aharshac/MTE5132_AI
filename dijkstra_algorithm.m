clear all;
clc;

% Define cell colors and color map (7x1 matrix)
CellColorMap = [ 
    [1 1 1];          % white  - clear
    [0 0 0];          % black  - obstacle
    [1 0 0];          % red    - visited
    [0 0 1];          % blue   - on list
    [0 1 0];          % green  - start
    [1 1 0];          % yellow - destination
    [0.5 0.5 0.5];    % grey   - path
];
% CellColorMap indexes (row index)
COLOR_CLEAR          = 1;    % white
COLOR_OBSTACLE       = 2;    % black
COLOR_VISITED        = 3;    % red
COLOR_ON_LIST        = 4;    % blue
COLOR_START          = 5;    % green
COLOR_DESTINATION    = 6;    % yellow
COLOR_PATH           = 7;    % grey

% constants
OBSTACLE = 1;
VISITED = 1;


% define size of square matrix
N = 10;
% Create graph
graph = false(N);
% Add obstacle
graph(3:9, 5:7) = OBSTACLE;


% Mark start and end nodes
% co-ordinates
coordStartNode = [6, 1];    
coordEndNode  = [8, 9];
% linear indexes
indexStartNode = sub2ind(size(graph), coordStartNode(1), coordStartNode(2));
indexEndNode = sub2ind(size(graph), coordEndNode(1), coordEndNode(2));


% Check if destination is not set in an obstacle cell
if graph(indexStartNode) == OBSTACLE || graph(indexEndNode) == OBSTACLE
    disp("Start and destination nodes cannot be obstacles");
    return;
end

% Define plot
img = zeros(N);  
img(graph == 0)     = COLOR_CLEAR;                  % Mark free cells
img(graph == 1)     = COLOR_OBSTACLE;               % Mark obstacle cells
img(indexStartNode) = COLOR_START;                  % Mark start
img(indexEndNode)   = COLOR_DESTINATION;            % Mark end

% Draw initial plot
figure(1);
colormap(CellColorMap);
image(1.5, 1.5, img);
grid on;
axis image;
title("START");
drawnow;
pause(1);


% Initialize distance, parent and visited arrays
distance = Inf(N);              % distance to each cell from start
parent = zeros(N);              % parent cell of each cell
visited = zeros(N);             % cells visited in previous iterations
distance(indexStartNode) = 0;   % distance of start cell w.r.t. itself is zero

while true
    colormap(CellColorMap);
    image(1.5, 1.5, img);
    grid on;
    axis image;
    title("Searching for goal...");
    drawnow;
    
    % find linear index of cell with least distance from start
    [~, indexMinCell] = min(distance(:));
    [i, j] = ind2sub(size(distance), indexMinCell); % co-ordinates of min cell
    
    % if min cell is destination, then break out of this loop
    if (indexMinCell == indexEndNode)
        break;
    end

    visited(i, j) = 1; % mark min cell as visited

    % matrix of cells adjacent to min cell
    % each row is a co-ordinate to one adjacent cell
    adjacent = [
        i-1, j;  % N
        i, j+1;  % E
        i, j-1;  % W
        i+1, j;  % S
    ];

    % iterate through each co-ordinate in adjacent cell
    for u = 1:4
        % co-ordinate of current adjacent cell
        p = adjacent(u, 1);       
        q = adjacent(u, 2);

        % check if all adjacent cell are withing the graph AND are not obstacles
        if (p > 0 && p <= N && q > 0 && q <= N) && (graph(p,q) ~= OBSTACLE)
            % if adjacent cell is not visited AND distance to adjacent cell > min cell
            if (visited(p,q) ~= VISITED && distance(p,q) > distance(i,j))
                    distance(p,q) = distance(i,j) + 1;  % increment distance of adjacent cell to min cell + 1
                    parent(p,q) = indexMinCell;         % set min cell as parent of adjacent cell
                    
                    % if adjacent cell is of visitable type, i.e, not obstacle, start, end, etc.
                    if (img(p,q) == COLOR_CLEAR) 
                        img(p,q) = COLOR_VISITED;   % mark adjacent cell as visited in image
                    end
            end
        end
    end
    
    % mark min cell as visited in distance matrix by setting to infinity
    distance(i,j) = Inf;
end

% Plot path to destination
if (isinf(distance(indexEndNode)))
    % min distance to destination node is infinity, i.e, path not found
    path = [];
    title("No path found");
else
    % form path by adding index of parent node successively from destination to start
    % parent node is pushed to beginning of current path, not end
    % path is a row vector whose no. of columns keep increasing when successive parents are added
    
    path = indexEndNode;    % start path from destination node
    
    % while first node in path has a parent
    while (parent(path(1)))
        % push the parent to beginning of current path
        path = [parent(path(1)), path];
    end
    
    % draw path vector by iteration, exluding start (first) and destination (last)
    for k = 2:length(path) - 1        
        img(path(k)) = COLOR_PATH;
        colormap(CellColorMap);
        image(1.5, 1.5, img);
        title("Drawing path from Start to End nodes...");
        grid on;
        axis image;
        pause(0.1);
    end
    title("FINISH");
end
