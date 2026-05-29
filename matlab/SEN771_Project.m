%% ========================================================================
%  SEN771 - Intelligent Autonomous Robots
%  Assignment 3: Trajectory Planning for Autonomous Mobile Robot
%  
%  Student Name : Dexter Leong
%  Student ID   : s223026243
%  Project Code : SEN771 T1 2026
%  
%  Description:
%  This program simulates autonomous operation of a mobile robot navigating
%  a 120m x 90m soccer field. The robot must visit 4 target locations while
%  avoiding 15 randomly placed obstacles, then return to the start.
%
%  Algorithms Implemented:
%    1. A* Search (covered in workshops)
%    2. Breadth-First Search (covered in workshops)
%    3. Rapidly-exploring Random Tree - RRT (new algorithm)
%
%  Features:
%    - Random obstacle generation with user-adjustable average size
%    - Grid-based meshing with occupied cell detection
%    - Target sequence optimisation (brute-force TSP)
%    - Three distinct path-finding algorithms with visual comparison
%    - Performance metrics (path length, computation time, nodes explored)
% ========================================================================

%% Initialisation
close all;
clc;
clear;

disp('==========================================================');
disp('  SEN771 - Autonomous Mobile Robot Trajectory Planning');
disp('  Student: Dexter Leong | ID: s223026243');
disp('==========================================================');
disp(' ');
pause(1);

%% ===================== USER PARAMETERS ==============================
% Obstacle average size (user adjustable)
% You can change this value or uncomment the input line below
% ObsAvgSize = input('Enter average obstacle size (e.g., 5-15): ');
ObsAvgSize = 10;  % Default average size in meters

% Mesh resolution (smaller = finer grid, slower computation)
MeshSize = 1;  % 1 meter cells

% Number of obstacles
NumObstacles = 15;

% Number of targets
NumTargets = 4;

% Random seed for reproducibility (comment out for truly random)
rng(42);

%% ===================== FIELD SETUP ==================================
% Standard soccer field: 120m x 90m
FieldLength = 120;  % x dimension
FieldWidth  = 90;   % y dimension

% Create main figure
MainFig = figure('Name', 'SEN771 - Robot Trajectory Planning', ...
    'Position', [50 50 1400 900]);

%% ===================== MESHING ======================================
disp('[1/7] Generating mesh...');

% Grid coordinates
FieldX = 0:MeshSize:FieldLength;
FieldY = 0:MeshSize:FieldWidth;

% Cell centres (used for path planning)
CellsX = FieldX(1:end-1) + MeshSize/2;
CellsY = FieldY(1:end-1) + MeshSize/2;

% Grid dimensions in cells
NumCellsX = length(CellsX);
NumCellsY = length(CellsY);

disp(['   Grid size: ', num2str(NumCellsX), ' x ', num2str(NumCellsY), ...
    ' = ', num2str(NumCellsX * NumCellsY), ' cells']);
disp(['   Mesh resolution: ', num2str(MeshSize), 'm']);

%% ===================== OBSTACLE GENERATION ==========================
disp('[2/7] Generating obstacles...');

% Obstacle map: 0 = free, 1 = obstacle
ObstacleMap = zeros(NumCellsX, NumCellsY);

% Store obstacle info for visualisation
ObstacleInfo = struct('x', {}, 'y', {}, 'width', {}, 'height', {});

% Generate 15 random obstacles with random shapes
for obs = 1:NumObstacles
    % Random width and height based on average size
    obsWidth  = ObsAvgSize * (0.5 + rand());   % 50%-150% of average
    obsHeight = ObsAvgSize * (0.5 + rand());
    
    % Random position (ensure obstacle stays within field)
    obsX = obsWidth/2 + rand() * (FieldLength - obsWidth);
    obsY = obsHeight/2 + rand() * (FieldWidth - obsHeight);
    
    % Store obstacle rectangle info
    ObstacleInfo(obs).x = obsX - obsWidth/2;
    ObstacleInfo(obs).y = obsY - obsHeight/2;
    ObstacleInfo(obs).width = obsWidth;
    ObstacleInfo(obs).height = obsHeight;
    
    % Mark cells occupied by this obstacle
    for i = 1:NumCellsX
        for j = 1:NumCellsY
            cx = CellsX(i);
            cy = CellsY(j);
            % Check if cell centre falls inside obstacle rectangle
            if cx >= (obsX - obsWidth/2) && cx <= (obsX + obsWidth/2) && ...
               cy >= (obsY - obsHeight/2) && cy <= (obsY + obsHeight/2)
                ObstacleMap(i, j) = 1;
            end
        end
    end
end

% Add safety margin around obstacles (inflate by 1 cell)
InflatedMap = ObstacleMap;
for i = 2:NumCellsX-1
    for j = 2:NumCellsY-1
        if ObstacleMap(i,j) == 1
            InflatedMap(max(1,i-1):min(NumCellsX,i+1), ...
                        max(1,j-1):min(NumCellsY,j+1)) = 1;
        end
    end
end

NumOccupied = sum(ObstacleMap(:));
disp(['   Obstacles generated: ', num2str(NumObstacles)]);
disp(['   Occupied cells: ', num2str(NumOccupied), ' / ', ...
    num2str(NumCellsX * NumCellsY)]);

%% ===================== ROBOT START POSITION =========================
% Robot starts at left edge, middle height
RobotStart = [1, round(NumCellsY/2)]';

% Ensure start position is not inside an obstacle
while InflatedMap(RobotStart(1), RobotStart(2)) == 1
    RobotStart(2) = RobotStart(2) + 1;
end

disp(['   Robot start: cell (', num2str(RobotStart(1)), ', ', ...
    num2str(RobotStart(2)), ') = (', ...
    num2str(CellsX(RobotStart(1))), 'm, ', ...
    num2str(CellsY(RobotStart(2))), 'm)']);

%% ===================== TARGET GENERATION ============================
disp('[3/7] Placing targets...');

% Generate 4 target positions in free space
TargetCells = zeros(2, NumTargets);
for t = 1:NumTargets
    validTarget = false;
    while ~validTarget
        tx = randi([round(NumCellsX*0.3), NumCellsX - 2]);
        ty = randi([3, NumCellsY - 2]);
        % Check not on obstacle and not too close to other targets
        if InflatedMap(tx, ty) == 0
            tooClose = false;
            for prev = 1:t-1
                if norm([tx; ty] - TargetCells(:, prev)) < 10
                    tooClose = true;
                    break;
                end
            end
            if ~tooClose
                validTarget = true;
            end
        end
    end
    TargetCells(:, t) = [tx; ty]';
end

for t = 1:NumTargets
    disp(['   Target ', num2str(t), ': cell (', ...
        num2str(TargetCells(1,t)), ', ', num2str(TargetCells(2,t)), ...
        ') = (', num2str(CellsX(TargetCells(1,t))), 'm, ', ...
        num2str(CellsY(TargetCells(2,t))), 'm)']);
end

%% ===================== VISUALISE SCENARIO ===========================
disp('[4/7] Visualising scenario...');

subplot(2, 3, 1);
hold on;
title('Scenario: Field, Obstacles & Targets');
xlabel('x (m)');
ylabel('y (m)');

% Draw field boundary
rectangle('Position', [0 0 FieldLength FieldWidth], ...
    'EdgeColor', 'b', 'LineWidth', 2);
axis([-5 FieldLength+5 -5 FieldWidth+5]);
axis equal;

% Draw grid lines (light gray)
for i = 0:MeshSize*10:FieldLength
    plot([i i], [0 FieldWidth], 'Color', [0.9 0.9 0.9]);
end
for j = 0:MeshSize*10:FieldWidth
    plot([0 FieldLength], [j j], 'Color', [0.9 0.9 0.9]);
end

% Draw obstacles (red rectangles)
for obs = 1:NumObstacles
    rectangle('Position', [ObstacleInfo(obs).x, ObstacleInfo(obs).y, ...
        ObstacleInfo(obs).width, ObstacleInfo(obs).height], ...
        'FaceColor', [1 0 0 0.7], 'EdgeColor', 'r', 'LineWidth', 1);
end

% Mark occupied cells (show grid occupation)
for i = 1:NumCellsX
    for j = 1:NumCellsY
        if ObstacleMap(i,j) == 1
            % Small red dot at cell centres occupied by obstacles
            plot(CellsX(i), CellsY(j), 'r.', 'MarkerSize', 2);
        end
    end
end

% Draw targets (green circles with labels)
for t = 1:NumTargets
    plot(CellsX(TargetCells(1,t)), CellsY(TargetCells(2,t)), ...
        'go', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'g');
    text(CellsX(TargetCells(1,t))+2, CellsY(TargetCells(2,t))+2, ...
        ['T', num2str(t)], 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'g');
end

% Draw robot start
plot(CellsX(RobotStart(1)), CellsY(RobotStart(2)), ...
    'ms', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'm');
text(CellsX(RobotStart(1))+2, CellsY(RobotStart(2))+2, ...
    'Start', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'm');

hold off;
drawnow;

%% ===================== TARGET SEQUENCE OPTIMISATION ==================
disp('[5/7] Optimising target visit sequence (TSP)...');

% Brute-force all permutations of 4 targets (4! = 24)
% Include return to start in total distance
Perms = perms(1:NumTargets);
BestDist = Inf;
BestOrder = [];

for p = 1:size(Perms, 1)
    order = Perms(p, :);
    totalDist = 0;
    
    % Distance from start to first target
    totalDist = totalDist + norm(RobotStart - TargetCells(:, order(1)));
    
    % Distance between consecutive targets
    for t = 1:NumTargets-1
        totalDist = totalDist + norm(TargetCells(:, order(t)) - TargetCells(:, order(t+1)));
    end
    
    % Distance from last target back to start
    totalDist = totalDist + norm(TargetCells(:, order(end)) - RobotStart);
    
    if totalDist < BestDist
        BestDist = totalDist;
        BestOrder = order;
    end
end

disp(['   Optimal target order: T', num2str(BestOrder(1)), ...
    ' -> T', num2str(BestOrder(2)), ' -> T', num2str(BestOrder(3)), ...
    ' -> T', num2str(BestOrder(4)), ' -> Start']);
disp(['   Estimated straight-line distance: ', num2str(round(BestDist)), ' cells']);

%% ===================== PATH FINDING FUNCTIONS ========================

% ---- Helper: Get valid neighbours (4-connected) ----
    function neighbours = getNeighbours(pos, mapSize, obstacleMap)
        neighbours = [];
        directions = [1 0; -1 0; 0 1; 0 -1]';  % right, left, up, down
        for d = 1:4
            newPos = pos + directions(:, d);
            if newPos(1) >= 1 && newPos(1) <= mapSize(1) && ...
               newPos(2) >= 1 && newPos(2) <= mapSize(2) && ...
               obstacleMap(newPos(1), newPos(2)) == 0
                neighbours = [neighbours, newPos];
            end
        end
    end

%% ===================== ALGORITHM 1: A* SEARCH ========================
disp('[6/7] Running path-finding algorithms...');
disp(' ');
disp('--- Algorithm 1: A* Search ---');

subplot(2, 3, 2);
hold on;
title('Algorithm 1: A* Search');
xlabel('x (m)'); ylabel('y (m)');
rectangle('Position', [0 0 FieldLength FieldWidth], 'EdgeColor', 'b', 'LineWidth', 2);
axis([-5 FieldLength+5 -5 FieldWidth+5]); axis equal;

% Draw obstacles
for obs = 1:NumObstacles
    rectangle('Position', [ObstacleInfo(obs).x, ObstacleInfo(obs).y, ...
        ObstacleInfo(obs).width, ObstacleInfo(obs).height], ...
        'FaceColor', [1 0 0 0.7], 'EdgeColor', 'r');
end
% Draw targets
for t = 1:NumTargets
    plot(CellsX(TargetCells(1,t)), CellsY(TargetCells(2,t)), ...
        'go', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'g');
end
plot(CellsX(RobotStart(1)), CellsY(RobotStart(2)), ...
    'ms', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'm');

% A* path finding function
tic;
AStarTotalPath = [];
AStarNodesExplored = 0;
AStarCurrentPos = RobotStart;

% Build waypoint list: start -> optimised targets -> start
Waypoints_AStar = [RobotStart, TargetCells(:, BestOrder), RobotStart];

PathColors = {'b', [0 0.6 0], [0.8 0.5 0], [0.5 0 0.5], [0 0.5 0.5]};

for wp = 1:size(Waypoints_AStar, 2) - 1
    startCell = Waypoints_AStar(:, wp);
    goalCell  = Waypoints_AStar(:, wp + 1);
    
    % A* implementation with open/closed lists
    openList  = startCell;
    gScore    = inf(NumCellsX, NumCellsY);
    fScore    = inf(NumCellsX, NumCellsY);
    cameFrom  = zeros(NumCellsX, NumCellsY, 2);
    closedMap = InflatedMap;  % treat obstacles as already closed
    
    gScore(startCell(1), startCell(2)) = 0;
    fScore(startCell(1), startCell(2)) = norm(startCell - goalCell);
    
    pathFound = false;
    
    while ~isempty(openList)
        % Find node in open list with lowest fScore
        bestF = Inf;
        bestIdx = 1;
        for k = 1:size(openList, 2)
            f = fScore(openList(1,k), openList(2,k));
            if f < bestF
                bestF = f;
                bestIdx = k;
            end
        end
        
        current = openList(:, bestIdx);
        openList = openList(:, [1:bestIdx-1, bestIdx+1:end]);
        AStarNodesExplored = AStarNodesExplored + 1;
        
        % Check if we reached the goal
        if norm(current - goalCell) == 0
            pathFound = true;
            break;
        end
        
        closedMap(current(1), current(2)) = 1;
        
        % Explore neighbours
        nbrs = getNeighbours(current, [NumCellsX, NumCellsY], closedMap);
        for n = 1:size(nbrs, 2)
            neighbour = nbrs(:, n);
            tentG = gScore(current(1), current(2)) + 1;
            
            if tentG < gScore(neighbour(1), neighbour(2))
                cameFrom(neighbour(1), neighbour(2), :) = current;
                gScore(neighbour(1), neighbour(2)) = tentG;
                fScore(neighbour(1), neighbour(2)) = tentG + norm(neighbour - goalCell);
                
                % Add to open list if not already there
                alreadyOpen = false;
                for k = 1:size(openList, 2)
                    if openList(1,k) == neighbour(1) && openList(2,k) == neighbour(2)
                        alreadyOpen = true;
                        break;
                    end
                end
                if ~alreadyOpen
                    openList = [openList, neighbour];
                end
            end
        end
    end
    
    % Reconstruct path
    if pathFound
        segmentPath = goalCell;
        node = goalCell;
        while ~(node(1) == startCell(1) && node(2) == startCell(2))
            prev = squeeze(cameFrom(node(1), node(2), :));
            segmentPath = [prev, segmentPath];
            node = prev;
        end
        
        % Plot this segment
        pathX = CellsX(segmentPath(1, :));
        pathY = CellsY(segmentPath(2, :));
        plot(pathX, pathY, '-', 'Color', PathColors{wp}, 'LineWidth', 2);
        
        AStarTotalPath = [AStarTotalPath, segmentPath];
    else
        disp(['   WARNING: A* could not find path for segment ', num2str(wp)]);
    end
end

AStarTime = toc;
AStarPathLen = size(AStarTotalPath, 2);

disp(['   Path length: ', num2str(AStarPathLen), ' cells (', ...
    num2str(AStarPathLen * MeshSize), 'm)']);
disp(['   Nodes explored: ', num2str(AStarNodesExplored)]);
disp(['   Computation time: ', num2str(round(AStarTime, 3)), 's']);

hold off;
drawnow;

%% ===================== ALGORITHM 2: BFS ==============================
disp(' ');
disp('--- Algorithm 2: Breadth-First Search ---');

subplot(2, 3, 3);
hold on;
title('Algorithm 2: Breadth-First Search');
xlabel('x (m)'); ylabel('y (m)');
rectangle('Position', [0 0 FieldLength FieldWidth], 'EdgeColor', 'b', 'LineWidth', 2);
axis([-5 FieldLength+5 -5 FieldWidth+5]); axis equal;

% Draw obstacles
for obs = 1:NumObstacles
    rectangle('Position', [ObstacleInfo(obs).x, ObstacleInfo(obs).y, ...
        ObstacleInfo(obs).width, ObstacleInfo(obs).height], ...
        'FaceColor', [1 0 0 0.7], 'EdgeColor', 'r');
end
for t = 1:NumTargets
    plot(CellsX(TargetCells(1,t)), CellsY(TargetCells(2,t)), ...
        'go', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'g');
end
plot(CellsX(RobotStart(1)), CellsY(RobotStart(2)), ...
    'ms', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'm');

tic;
BFSTotalPath = [];
BFSNodesExplored = 0;

Waypoints_BFS = [RobotStart, TargetCells(:, BestOrder), RobotStart];

for wp = 1:size(Waypoints_BFS, 2) - 1
    startCell = Waypoints_BFS(:, wp);
    goalCell  = Waypoints_BFS(:, wp + 1);
    
    % BFS implementation
    queue = startCell;
    visited = InflatedMap;  % obstacles pre-marked
    visited(startCell(1), startCell(2)) = 1;
    cameFrom = zeros(NumCellsX, NumCellsY, 2);
    
    pathFound = false;
    
    while ~isempty(queue)
        current = queue(:, 1);
        queue = queue(:, 2:end);
        BFSNodesExplored = BFSNodesExplored + 1;
        
        if norm(current - goalCell) == 0
            pathFound = true;
            break;
        end
        
        nbrs = getNeighbours(current, [NumCellsX, NumCellsY], visited);
        for n = 1:size(nbrs, 2)
            neighbour = nbrs(:, n);
            if visited(neighbour(1), neighbour(2)) == 0
                visited(neighbour(1), neighbour(2)) = 1;
                cameFrom(neighbour(1), neighbour(2), :) = current;
                queue = [queue, neighbour];
            end
        end
    end
    
    % Reconstruct path
    if pathFound
        segmentPath = goalCell;
        node = goalCell;
        while ~(node(1) == startCell(1) && node(2) == startCell(2))
            prev = squeeze(cameFrom(node(1), node(2), :));
            segmentPath = [prev, segmentPath];
            node = prev;
        end
        
        pathX = CellsX(segmentPath(1, :));
        pathY = CellsY(segmentPath(2, :));
        plot(pathX, pathY, '-', 'Color', PathColors{wp}, 'LineWidth', 2);
        
        BFSTotalPath = [BFSTotalPath, segmentPath];
    else
        disp(['   WARNING: BFS could not find path for segment ', num2str(wp)]);
    end
end

BFSTime = toc;
BFSPathLen = size(BFSTotalPath, 2);

disp(['   Path length: ', num2str(BFSPathLen), ' cells (', ...
    num2str(BFSPathLen * MeshSize), 'm)']);
disp(['   Nodes explored: ', num2str(BFSNodesExplored)]);
disp(['   Computation time: ', num2str(round(BFSTime, 3)), 's']);

hold off;
drawnow;

%% ===================== ALGORITHM 3: RRT (NEW) ========================
disp(' ');
disp('--- Algorithm 3: Rapidly-exploring Random Tree (RRT) ---');
disp('   (New algorithm - not covered in workshops)');

subplot(2, 3, 4);
hold on;
title('Algorithm 3: RRT (New Algorithm)');
xlabel('x (m)'); ylabel('y (m)');
rectangle('Position', [0 0 FieldLength FieldWidth], 'EdgeColor', 'b', 'LineWidth', 2);
axis([-5 FieldLength+5 -5 FieldWidth+5]); axis equal;

% Draw obstacles
for obs = 1:NumObstacles
    rectangle('Position', [ObstacleInfo(obs).x, ObstacleInfo(obs).y, ...
        ObstacleInfo(obs).width, ObstacleInfo(obs).height], ...
        'FaceColor', [1 0 0 0.7], 'EdgeColor', 'r');
end
for t = 1:NumTargets
    plot(CellsX(TargetCells(1,t)), CellsY(TargetCells(2,t)), ...
        'go', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'g');
end
plot(CellsX(RobotStart(1)), CellsY(RobotStart(2)), ...
    'ms', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'm');

tic;
RRTTotalPath = [];
RRTNodesExplored = 0;
MaxRRTIterations = 15000;
StepSize = 3;  % cells per step
GoalBias = 0.15;  % 15% chance of sampling goal directly

Waypoints_RRT = [RobotStart, TargetCells(:, BestOrder), RobotStart];

for wp = 1:size(Waypoints_RRT, 2) - 1
    startCell = Waypoints_RRT(:, wp);
    goalCell  = Waypoints_RRT(:, wp + 1);
    
    % RRT implementation
    treeNodes = startCell;        % columns are nodes
    treeParent = 0;               % parent index (0 = root)
    
    pathFound = false;
    goalNodeIdx = -1;
    
    for iter = 1:MaxRRTIterations
        % Sample random point (with goal bias)
        if rand() < GoalBias
            randPoint = goalCell;
        else
            randPoint = [randi([1, NumCellsX]); randi([1, NumCellsY])];
        end
        
        % Find nearest node in tree
        dists = vecnorm(treeNodes - randPoint);
        [~, nearestIdx] = min(dists);
        nearestNode = treeNodes(:, nearestIdx);
        
        % Steer towards random point
        direction = randPoint - nearestNode;
        dirNorm = norm(direction);
        if dirNorm == 0
            continue;
        end
        direction = direction / dirNorm;
        newNode = round(nearestNode + direction * min(StepSize, dirNorm));
        
        % Clamp to field bounds
        newNode(1) = max(1, min(NumCellsX, newNode(1)));
        newNode(2) = max(1, min(NumCellsY, newNode(2)));
        
        % Check if path to new node is collision-free
        collisionFree = true;
        numChecks = max(abs(newNode - nearestNode));
        if numChecks > 0
            for c = 0:numChecks
                checkPoint = round(nearestNode + (newNode - nearestNode) * c / numChecks);
                checkPoint(1) = max(1, min(NumCellsX, checkPoint(1)));
                checkPoint(2) = max(1, min(NumCellsY, checkPoint(2)));
                if InflatedMap(checkPoint(1), checkPoint(2)) == 1
                    collisionFree = false;
                    break;
                end
            end
        end
        
        if collisionFree
            treeNodes = [treeNodes, newNode];
            treeParent = [treeParent, nearestIdx];
            RRTNodesExplored = RRTNodesExplored + 1;
            
            % Draw tree branch (light gray)
            plot([CellsX(nearestNode(1)), CellsX(newNode(1))], ...
                 [CellsY(nearestNode(2)), CellsY(newNode(2))], ...
                 '-', 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5);
            
            % Check if we reached the goal
            if norm(newNode - goalCell) <= StepSize
                % Connect directly to goal if collision-free
                numChecks2 = max(abs(goalCell - newNode));
                goalFree = true;
                if numChecks2 > 0
                    for c = 0:numChecks2
                        cp = round(newNode + (goalCell - newNode) * c / numChecks2);
                        cp(1) = max(1, min(NumCellsX, cp(1)));
                        cp(2) = max(1, min(NumCellsY, cp(2)));
                        if InflatedMap(cp(1), cp(2)) == 1
                            goalFree = false;
                            break;
                        end
                    end
                end
                if goalFree
                    treeNodes = [treeNodes, goalCell];
                    treeParent = [treeParent, size(treeNodes, 2) - 1];
                    goalNodeIdx = size(treeNodes, 2);
                    pathFound = true;
                    break;
                end
            end
        end
    end
    
    % Reconstruct path from tree
    if pathFound
        segmentPath = [];
        idx = goalNodeIdx;
        while idx > 0
            segmentPath = [treeNodes(:, idx), segmentPath];
            idx = treeParent(idx);
        end
        
        pathX = CellsX(segmentPath(1, :));
        pathY = CellsY(segmentPath(2, :));
        plot(pathX, pathY, '-', 'Color', PathColors{wp}, 'LineWidth', 2.5);
        
        RRTTotalPath = [RRTTotalPath, segmentPath];
    else
        disp(['   WARNING: RRT could not find path for segment ', num2str(wp), ...
            ' after ', num2str(MaxRRTIterations), ' iterations']);
    end
end

RRTTime = toc;
RRTPathLen = size(RRTTotalPath, 2);

disp(['   Path length: ', num2str(RRTPathLen), ' cells']);
disp(['   Nodes explored: ', num2str(RRTNodesExplored)]);
disp(['   Computation time: ', num2str(round(RRTTime, 3)), 's']);

hold off;
drawnow;

%% ===================== ROBOT MOVEMENT SIMULATION =====================
disp(' ');
disp('[7/7] Simulating robot movement (A* path)...');

subplot(2, 3, 5);
hold on;
title('Robot Movement Simulation (A* Path)');
xlabel('x (m)'); ylabel('y (m)');
rectangle('Position', [0 0 FieldLength FieldWidth], 'EdgeColor', 'b', 'LineWidth', 2);
axis([-5 FieldLength+5 -5 FieldWidth+5]); axis equal;

% Draw obstacles
for obs = 1:NumObstacles
    rectangle('Position', [ObstacleInfo(obs).x, ObstacleInfo(obs).y, ...
        ObstacleInfo(obs).width, ObstacleInfo(obs).height], ...
        'FaceColor', [1 0 0 0.7], 'EdgeColor', 'r');
end
for t = 1:NumTargets
    plot(CellsX(TargetCells(1,t)), CellsY(TargetCells(2,t)), ...
        'go', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'g');
end

% Simulate movement with time steps
dT = 0.1;  % time step
RobotSpeed = 2;  % cells per second

if ~isempty(AStarTotalPath)
    robotHandle = plot(CellsX(AStarTotalPath(1,1)), ...
        CellsY(AStarTotalPath(2,1)), 'ms', 'MarkerSize', 12, ...
        'LineWidth', 2, 'MarkerFaceColor', 'm');
    trailX = [];
    trailY = [];
    
    % Animate every Nth step for speed
    stepSkip = max(1, round(1 / (RobotSpeed * dT)));
    
    for step = 1:stepSkip:size(AStarTotalPath, 2)
        px = CellsX(AStarTotalPath(1, step));
        py = CellsY(AStarTotalPath(2, step));
        
        trailX = [trailX, px];
        trailY = [trailY, py];
        
        delete(robotHandle);
        plot(trailX, trailY, 'b-', 'LineWidth', 1.5);
        robotHandle = plot(px, py, 'ms', 'MarkerSize', 12, ...
            'LineWidth', 2, 'MarkerFaceColor', 'm');
        
        drawnow;
        pause(0.01);
    end
    
    % Ensure final position is shown
    plot(trailX, trailY, 'b-', 'LineWidth', 1.5);
    plot(CellsX(AStarTotalPath(1,end)), CellsY(AStarTotalPath(2,end)), ...
        'ms', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'm');
end

hold off;
drawnow;

%% ===================== PERFORMANCE COMPARISON ========================
disp(' ');
disp('=========== PERFORMANCE COMPARISON ===========');
disp(' ');
fprintf('%-15s %-15s %-18s %-15s\n', 'Algorithm', 'Path Length', 'Nodes Explored', 'Time (s)');
disp('--------------------------------------------------------------');
fprintf('%-15s %-15s %-18s %-15s\n', 'A*', ...
    [num2str(AStarPathLen), ' cells'], num2str(AStarNodesExplored), num2str(round(AStarTime, 3)));
fprintf('%-15s %-15s %-18s %-15s\n', 'BFS', ...
    [num2str(BFSPathLen), ' cells'], num2str(BFSNodesExplored), num2str(round(BFSTime, 3)));
fprintf('%-15s %-15s %-18s %-15s\n', 'RRT', ...
    [num2str(RRTPathLen), ' cells'], num2str(RRTNodesExplored), num2str(round(RRTTime, 3)));
disp('--------------------------------------------------------------');
disp(' ');

% Performance comparison bar chart
subplot(2, 3, 6);
algNames = {'A*', 'BFS', 'RRT'};
pathLens = [AStarPathLen, BFSPathLen, RRTPathLen];
bar(pathLens);
set(gca, 'XTickLabel', algNames);
ylabel('Total Path Length (cells)');
title('Algorithm Comparison: Path Length');
grid on;

% Add text labels on bars
for b = 1:3
    text(b, pathLens(b) + max(pathLens)*0.02, num2str(pathLens(b)), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end

%% ===================== SUMMARY ======================================
disp('==========================================================');
disp('  Project completed successfully!');
disp(['  Target visit order: T', num2str(BestOrder(1)), ...
    ' -> T', num2str(BestOrder(2)), ' -> T', num2str(BestOrder(3)), ...
    ' -> T', num2str(BestOrder(4)), ' -> Start']);
disp(['  Best algorithm (shortest path): A* with ', num2str(AStarPathLen), ' cells']);
disp('==========================================================');
