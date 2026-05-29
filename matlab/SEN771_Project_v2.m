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
%  Obstacles are randomly generated each run based on user-adjustable size.
%
%  Algorithms Implemented:
%    1. A* Search (covered in workshops)
%    2. Breadth-First Search (covered in workshops)
%    3. Rapidly-exploring Random Tree - RRT (new algorithm)
%
%  Features:
%    - Random obstacle generation (different each run)
%    - User-adjustable average obstacle size
%    - Grid-based meshing with occupied cell detection
%    - Target sequence optimisation (brute-force TSP)
%    - Three path-finding algorithms with animated movement
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

%% ===================== USER PARAMETERS ==============================
% Obstacle average size - user can adjust this each run
% Uncomment the line below to enter manually each time:
% ObsAvgSize = input('Enter average obstacle size (e.g., 5-15): ');
ObsAvgSize = 10;  % Default average size in meters

% Mesh resolution (smaller = finer grid, slower computation)
MeshSize = 1;  % 1 meter cells

% Number of obstacles
NumObstacles = 15;

% Number of targets
NumTargets = 4;

% Animation speed control (higher = faster, set 0 to skip animation)
AnimSpeed = 5;  % show every Nth step

% NOTE: No rng() seed is set, so obstacles are random each run.
% To reproduce a specific layout for testing, uncomment below:
% rng(42);

%% ===================== FIELD SETUP ==================================
% Standard soccer field: 120m x 90m
FieldLength = 120;  % x dimension
FieldWidth  = 90;   % y dimension

%% ===================== MESHING ======================================
disp('[1/7] Generating mesh...');

% Grid edge coordinates
FieldX = 0:MeshSize:FieldLength;
FieldY = 0:MeshSize:FieldWidth;

% Cell centre coordinates (all planning uses these)
CellsX = FieldX(1:end-1) + MeshSize/2;
CellsY = FieldY(1:end-1) + MeshSize/2;

% Grid dimensions in cells
NumCellsX = length(CellsX);
NumCellsY = length(CellsY);

disp(['   Grid size: ', num2str(NumCellsX), ' x ', num2str(NumCellsY), ...
    ' = ', num2str(NumCellsX * NumCellsY), ' cells']);

%% ===================== OBSTACLE GENERATION ==========================
disp('[2/7] Generating random obstacles...');

% Obstacle occupancy map: 0 = free, 1 = obstacle
ObstacleMap = zeros(NumCellsX, NumCellsY);

% Store obstacle rectangles for drawing
ObsRect = zeros(NumObstacles, 4);  % [x, y, width, height] per obstacle

for obs = 1:NumObstacles
    % Random width and height based on user's average size
    w = ObsAvgSize * (0.5 + rand());
    h = ObsAvgSize * (0.5 + rand());
    
    % Random position (keep obstacle fully inside the field)
    ox = w/2 + rand() * (FieldLength - w);
    oy = h/2 + rand() * (FieldWidth - h);
    
    % Store rectangle [bottom-left x, bottom-left y, width, height]
    ObsRect(obs, :) = [ox - w/2, oy - h/2, w, h];
    
    % Mark grid cells whose centres fall inside this obstacle
    xMin = ox - w/2;  xMax = ox + w/2;
    yMin = oy - h/2;  yMax = oy + h/2;
    
    % Find cell index ranges to avoid checking every cell
    iMin = max(1, floor(xMin / MeshSize));
    iMax = min(NumCellsX, ceil(xMax / MeshSize));
    jMin = max(1, floor(yMin / MeshSize));
    jMax = min(NumCellsY, ceil(yMax / MeshSize));
    
    for i = iMin:iMax
        for j = jMin:jMax
            if CellsX(i) >= xMin && CellsX(i) <= xMax && ...
               CellsY(j) >= yMin && CellsY(j) <= yMax
                ObstacleMap(i, j) = 1;
            end
        end
    end
end

% Safety margin: inflate obstacles by 1 cell in all directions
InflatedMap = ObstacleMap;
for i = 1:NumCellsX
    for j = 1:NumCellsY
        if ObstacleMap(i,j) == 1
            for di = -1:1
                for dj = -1:1
                    ni = i + di;  nj = j + dj;
                    if ni >= 1 && ni <= NumCellsX && nj >= 1 && nj <= NumCellsY
                        InflatedMap(ni, nj) = 1;
                    end
                end
            end
        end
    end
end

disp(['   Obstacles: ', num2str(NumObstacles), ...
    ' | Occupied cells: ', num2str(sum(ObstacleMap(:))), ...
    ' | Inflated cells: ', num2str(sum(InflatedMap(:)))]);

%% ===================== ROBOT START ==================================
RobotStart = [1, round(NumCellsY/2)]';

% Shift if start is inside an obstacle
while InflatedMap(RobotStart(1), RobotStart(2)) == 1
    RobotStart(2) = RobotStart(2) + 1;
    if RobotStart(2) > NumCellsY
        RobotStart(2) = 1;  % wrap around if needed
    end
end

disp(['   Robot start: (', num2str(CellsX(RobotStart(1))), ...
    'm, ', num2str(CellsY(RobotStart(2))), 'm)']);

%% ===================== TARGET GENERATION ============================
disp('[3/7] Placing targets randomly...');

TargetCells = zeros(2, NumTargets);
for t = 1:NumTargets
    placed = false;
    while ~placed
        tx = randi([round(NumCellsX * 0.3), NumCellsX - 2]);
        ty = randi([3, NumCellsY - 2]);
        if InflatedMap(tx, ty) == 0
            tooClose = false;
            for prev = 1:t-1
                if norm([tx; ty] - TargetCells(:, prev)) < 10
                    tooClose = true; break;
                end
            end
            if ~tooClose
                TargetCells(:, t) = [tx; ty];
                placed = true;
            end
        end
    end
    disp(['   Target ', num2str(t), ': (', ...
        num2str(CellsX(tx)), 'm, ', num2str(CellsY(ty)), 'm)']);
end

%% ===================== VISUALISE SCENARIO ===========================
disp('[4/7] Drawing scenario...');

figure('Name', 'SEN771 - Scenario Overview', 'Position', [50 50 800 600]);
hold on;
title('Soccer Field - Obstacles, Targets & Robot Start');
xlabel('x (m)'); ylabel('y (m)');
rectangle('Position', [0 0 FieldLength FieldWidth], 'EdgeColor', 'b', 'LineWidth', 2);
axis([-5 FieldLength+5 -5 FieldWidth+5]); axis equal;

% Grid lines every 10m
for i = 0:10:FieldLength
    plot([i i], [0 FieldWidth], 'Color', [0.9 0.9 0.9]);
end
for j = 0:10:FieldWidth
    plot([0 FieldLength], [j j], 'Color', [0.9 0.9 0.9]);
end

% Obstacles (red)
for obs = 1:NumObstacles
    rectangle('Position', ObsRect(obs,:), ...
        'FaceColor', [1 0 0 0.7], 'EdgeColor', 'r');
end

% Occupied cell markers
for i = 1:NumCellsX
    for j = 1:NumCellsY
        if ObstacleMap(i,j) == 1
            plot(CellsX(i), CellsY(j), 'r.', 'MarkerSize', 3);
        end
    end
end

% Targets (green)
for t = 1:NumTargets
    plot(CellsX(TargetCells(1,t)), CellsY(TargetCells(2,t)), ...
        'go', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'g');
    text(CellsX(TargetCells(1,t))+2, CellsY(TargetCells(2,t))+2, ...
        ['T', num2str(t)], 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0.5 0]);
end

% Robot start (magenta)
plot(CellsX(RobotStart(1)), CellsY(RobotStart(2)), ...
    'ms', 'MarkerSize', 12, 'LineWidth', 2, 'MarkerFaceColor', 'm');
text(CellsX(RobotStart(1))+2, CellsY(RobotStart(2))+2, ...
    'Start', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'm');
hold off;
drawnow;

%% ===================== TARGET SEQUENCE OPTIMISATION ==================
disp('[5/7] Optimising target visit order (TSP brute-force)...');

Perms = perms(1:NumTargets);
BestDist = Inf;
BestOrder = [];

for p = 1:size(Perms, 1)
    order = Perms(p, :);
    d = norm(RobotStart - TargetCells(:, order(1)));
    for t = 1:NumTargets-1
        d = d + norm(TargetCells(:, order(t)) - TargetCells(:, order(t+1)));
    end
    d = d + norm(TargetCells(:, order(end)) - RobotStart);
    if d < BestDist
        BestDist = d;
        BestOrder = order;
    end
end

disp(['   Best order: Start -> T', num2str(BestOrder(1)), ...
    ' -> T', num2str(BestOrder(2)), ' -> T', num2str(BestOrder(3)), ...
    ' -> T', num2str(BestOrder(4)), ' -> Start']);

%% ===================== ALGORITHM 1: A* SEARCH ========================
disp(' ');
disp('[6/7] Running path-finding algorithms...');
disp(' ');
disp('--- Algorithm 1: A* Search ---');

tic;
AStarTotalPath = [];
AStarNodesExplored = 0;

Waypoints = [RobotStart, TargetCells(:, BestOrder), RobotStart];
SegColors = {'b', [0 0.6 0], [0.8 0.5 0], [0.5 0 0.5], [0 0.5 0.5]};

for wp = 1:size(Waypoints, 2) - 1
    sCell = Waypoints(:, wp);
    gCell = Waypoints(:, wp + 1);
    
    % A* with open/closed lists, gScore, fScore, cameFrom
    openList  = sCell;
    gScore    = inf(NumCellsX, NumCellsY);
    fScore    = inf(NumCellsX, NumCellsY);
    cameFrom  = zeros(NumCellsX, NumCellsY, 2);
    closed    = InflatedMap;
    
    gScore(sCell(1), sCell(2)) = 0;
    fScore(sCell(1), sCell(2)) = norm(sCell - gCell);
    found = false;
    
    while ~isempty(openList)
        % Pick node with lowest fScore
        bestF = Inf; bestIdx = 1;
        for k = 1:size(openList, 2)
            f = fScore(openList(1,k), openList(2,k));
            if f < bestF; bestF = f; bestIdx = k; end
        end
        cur = openList(:, bestIdx);
        openList = openList(:, [1:bestIdx-1, bestIdx+1:end]);
        AStarNodesExplored = AStarNodesExplored + 1;
        
        if cur(1)==gCell(1) && cur(2)==gCell(2); found = true; break; end
        closed(cur(1), cur(2)) = 1;
        
        % Check 4 neighbours
        dirs = [1 0; -1 0; 0 1; 0 -1]';
        for d = 1:4
            nb = cur + dirs(:,d);
            if nb(1)<1||nb(1)>NumCellsX||nb(2)<1||nb(2)>NumCellsY; continue; end
            if closed(nb(1),nb(2))==1; continue; end
            
            tentG = gScore(cur(1),cur(2)) + 1;
            if tentG < gScore(nb(1),nb(2))
                cameFrom(nb(1),nb(2),:) = cur;
                gScore(nb(1),nb(2)) = tentG;
                fScore(nb(1),nb(2)) = tentG + norm(nb - gCell);
                % Add to open if not already there
                inOpen = false;
                for k = 1:size(openList,2)
                    if openList(1,k)==nb(1) && openList(2,k)==nb(2)
                        inOpen = true; break;
                    end
                end
                if ~inOpen; openList = [openList, nb]; end
            end
        end
    end
    
    % Reconstruct path
    if found
        seg = gCell;
        node = gCell;
        while ~(node(1)==sCell(1) && node(2)==sCell(2))
            node = squeeze(cameFrom(node(1),node(2),:));
            seg = [node, seg];
        end
        AStarTotalPath = [AStarTotalPath, seg];
    else
        disp(['   WARNING: No path for segment ', num2str(wp)]);
    end
end
AStarTime = toc;
AStarLen = size(AStarTotalPath, 2);
disp(['   Path: ', num2str(AStarLen), ' cells | Nodes: ', ...
    num2str(AStarNodesExplored), ' | Time: ', num2str(round(AStarTime,3)), 's']);

%% ===================== ALGORITHM 2: BFS ==============================
disp(' ');
disp('--- Algorithm 2: Breadth-First Search ---');

tic;
BFSTotalPath = [];
BFSNodesExplored = 0;

for wp = 1:size(Waypoints, 2) - 1
    sCell = Waypoints(:, wp);
    gCell = Waypoints(:, wp + 1);
    
    queue = sCell;
    visited = InflatedMap;
    visited(sCell(1), sCell(2)) = 1;
    cameFrom = zeros(NumCellsX, NumCellsY, 2);
    found = false;
    
    while ~isempty(queue)
        cur = queue(:, 1);
        queue = queue(:, 2:end);
        BFSNodesExplored = BFSNodesExplored + 1;
        
        if cur(1)==gCell(1) && cur(2)==gCell(2); found = true; break; end
        
        dirs = [1 0; -1 0; 0 1; 0 -1]';
        for d = 1:4
            nb = cur + dirs(:,d);
            if nb(1)<1||nb(1)>NumCellsX||nb(2)<1||nb(2)>NumCellsY; continue; end
            if visited(nb(1),nb(2))==1; continue; end
            visited(nb(1),nb(2)) = 1;
            cameFrom(nb(1),nb(2),:) = cur;
            queue = [queue, nb];
        end
    end
    
    if found
        seg = gCell;
        node = gCell;
        while ~(node(1)==sCell(1) && node(2)==sCell(2))
            node = squeeze(cameFrom(node(1),node(2),:));
            seg = [node, seg];
        end
        BFSTotalPath = [BFSTotalPath, seg];
    else
        disp(['   WARNING: No path for segment ', num2str(wp)]);
    end
end
BFSTime = toc;
BFSLen = size(BFSTotalPath, 2);
disp(['   Path: ', num2str(BFSLen), ' cells | Nodes: ', ...
    num2str(BFSNodesExplored), ' | Time: ', num2str(round(BFSTime,3)), 's']);

%% ===================== ALGORITHM 3: RRT (NEW) ========================
disp(' ');
disp('--- Algorithm 3: RRT (Rapidly-exploring Random Tree) ---');
disp('   [NEW - not covered in workshops]');

tic;
RRTTotalPath = [];
RRTNodesExplored = 0;
MaxIter = 15000;
StepSize = 3;
GoalBias = 0.15;

for wp = 1:size(Waypoints, 2) - 1
    sCell = Waypoints(:, wp);
    gCell = Waypoints(:, wp + 1);
    
    tree = sCell;
    parent = 0;
    found = false;
    goalIdx = -1;
    
    for iter = 1:MaxIter
        % Sample (biased toward goal)
        if rand() < GoalBias
            rndPt = gCell;
        else
            rndPt = [randi([1 NumCellsX]); randi([1 NumCellsY])];
        end
        
        % Nearest node in tree
        dists = vecnorm(tree - rndPt);
        [~, nearIdx] = min(dists);
        nearNode = tree(:, nearIdx);
        
        % Steer toward sample
        dir = rndPt - nearNode;
        d = norm(dir);
        if d == 0; continue; end
        newNode = round(nearNode + (dir/d) * min(StepSize, d));
        newNode(1) = max(1, min(NumCellsX, newNode(1)));
        newNode(2) = max(1, min(NumCellsY, newNode(2)));
        
        % Collision check along line
        nChecks = max(abs(newNode - nearNode));
        clear_path = true;
        if nChecks > 0
            for c = 0:nChecks
                cp = round(nearNode + (newNode - nearNode) * c / nChecks);
                cp(1) = max(1, min(NumCellsX, cp(1)));
                cp(2) = max(1, min(NumCellsY, cp(2)));
                if InflatedMap(cp(1), cp(2)) == 1
                    clear_path = false; break;
                end
            end
        end
        
        if clear_path
            tree = [tree, newNode];
            parent = [parent, nearIdx];
            RRTNodesExplored = RRTNodesExplored + 1;
            
            % Check if close enough to connect to goal
            if norm(newNode - gCell) <= StepSize
                nC2 = max(abs(gCell - newNode));
                gFree = true;
                if nC2 > 0
                    for c = 0:nC2
                        cp = round(newNode + (gCell - newNode) * c / nC2);
                        cp(1) = max(1, min(NumCellsX, cp(1)));
                        cp(2) = max(1, min(NumCellsY, cp(2)));
                        if InflatedMap(cp(1),cp(2)) == 1
                            gFree = false; break;
                        end
                    end
                end
                if gFree
                    tree = [tree, gCell];
                    parent = [parent, size(tree,2)-1];
                    goalIdx = size(tree, 2);
                    found = true; break;
                end
            end
        end
    end
    
    if found
        seg = [];
        idx = goalIdx;
        while idx > 0
            seg = [tree(:,idx), seg];
            idx = parent(idx);
        end
        RRTTotalPath = [RRTTotalPath, seg];
    else
        disp(['   WARNING: No path for segment ', num2str(wp)]);
    end
end
RRTTime = toc;
RRTLen = size(RRTTotalPath, 2);
disp(['   Path: ', num2str(RRTLen), ' cells | Nodes: ', ...
    num2str(RRTNodesExplored), ' | Time: ', num2str(round(RRTTime,3)), 's']);

%% ===================== ANIMATED MOVEMENT - ALL 3 ALGORITHMS ==========
disp(' ');
disp('[7/7] Animating robot movement for all algorithms...');

algNames  = {'A* Search', 'Breadth-First Search', 'RRT (New Algorithm)'};
algPaths  = {AStarTotalPath, BFSTotalPath, RRTTotalPath};
algColors = {'b', [0 0.7 0], [0.8 0.4 0]};

figure('Name', 'SEN771 - Robot Movement Animation', 'Position', [50 50 1500 450]);

for alg = 1:3
    subplot(1, 3, alg);
    hold on;
    title([algNames{alg}, ' - Movement']);
    xlabel('x (m)'); ylabel('y (m)');
    rectangle('Position', [0 0 FieldLength FieldWidth], ...
        'EdgeColor', 'b', 'LineWidth', 2);
    axis([-5 FieldLength+5 -5 FieldWidth+5]); axis equal;
    
    % Draw obstacles
    for obs = 1:NumObstacles
        rectangle('Position', ObsRect(obs,:), ...
            'FaceColor', [1 0 0 0.7], 'EdgeColor', 'r');
    end
    
    % Draw targets
    for t = 1:NumTargets
        plot(CellsX(TargetCells(1,t)), CellsY(TargetCells(2,t)), ...
            'go', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'g');
        text(CellsX(TargetCells(1,t))+1, CellsY(TargetCells(2,t))+1, ...
            ['T', num2str(t)], 'FontSize', 8, 'Color', [0 0.5 0]);
    end
    
    % Draw start
    plot(CellsX(RobotStart(1)), CellsY(RobotStart(2)), ...
        'ms', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'm');
end
drawnow;

% Animate all three simultaneously
maxSteps = max([size(AStarTotalPath,2), size(BFSTotalPath,2), size(RRTTotalPath,2)]);
robotHandles = gobjects(1, 3);

% Place initial robot markers
for alg = 1:3
    if ~isempty(algPaths{alg})
        subplot(1, 3, alg);
        robotHandles(alg) = plot(CellsX(algPaths{alg}(1,1)), ...
            CellsY(algPaths{alg}(2,1)), 'ms', 'MarkerSize', 10, ...
            'LineWidth', 2, 'MarkerFaceColor', 'm');
    end
end

% Run animation
trailHandles = gobjects(1, 3);
trailX = cell(1,3);  trailY = cell(1,3);
for alg = 1:3; trailX{alg} = []; trailY{alg} = []; end

step = 1;
while step <= maxSteps
    for alg = 1:3
        path = algPaths{alg};
        if isempty(path); continue; end
        
        % Clamp step to this path's length
        s = min(step, size(path, 2));
        px = CellsX(path(1, s));
        py = CellsY(path(2, s));
        
        trailX{alg} = [trailX{alg}, px];
        trailY{alg} = [trailY{alg}, py];
        
        subplot(1, 3, alg);
        % Update trail
        if isvalid(trailHandles(alg))
            delete(trailHandles(alg));
        end
        trailHandles(alg) = plot(trailX{alg}, trailY{alg}, '-', ...
            'Color', algColors{alg}, 'LineWidth', 1.5);
        
        % Update robot position
        if isvalid(robotHandles(alg))
            delete(robotHandles(alg));
        end
        robotHandles(alg) = plot(px, py, 'ms', 'MarkerSize', 10, ...
            'LineWidth', 2, 'MarkerFaceColor', 'm');
    end
    
    drawnow;
    pause(0.005);
    step = step + AnimSpeed;
end

% Draw final positions
for alg = 1:3
    path = algPaths{alg};
    if isempty(path); continue; end
    subplot(1, 3, alg);
    plot(CellsX(path(1,:)), CellsY(path(2,:)), '-', ...
        'Color', algColors{alg}, 'LineWidth', 2);
    plot(CellsX(path(1,end)), CellsY(path(2,end)), ...
        'ms', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'm');
end
drawnow;

%% ===================== PERFORMANCE COMPARISON ========================
disp(' ');
disp('============= PERFORMANCE COMPARISON =============');
disp(' ');
fprintf('%-12s %-18s %-18s %-12s\n', 'Algorithm', 'Path (cells)', 'Nodes Explored', 'Time (s)');
disp('------------------------------------------------------------');
fprintf('%-12s %-18d %-18d %-12.3f\n', 'A*',  AStarLen, AStarNodesExplored, AStarTime);
fprintf('%-12s %-18d %-18d %-12.3f\n', 'BFS', BFSLen,   BFSNodesExplored,   BFSTime);
fprintf('%-12s %-18d %-18d %-12.3f\n', 'RRT', RRTLen,   RRTNodesExplored,   RRTTime);
disp('------------------------------------------------------------');

% Bar chart comparison
figure('Name', 'SEN771 - Performance Comparison', 'Position', [100 100 900 400]);

subplot(1, 3, 1);
bar([AStarLen, BFSLen, RRTLen]);
set(gca, 'XTickLabel', {'A*', 'BFS', 'RRT'});
ylabel('Cells'); title('Path Length'); grid on;

subplot(1, 3, 2);
bar([AStarNodesExplored, BFSNodesExplored, RRTNodesExplored]);
set(gca, 'XTickLabel', {'A*', 'BFS', 'RRT'});
ylabel('Nodes'); title('Nodes Explored'); grid on;

subplot(1, 3, 3);
bar([AStarTime, BFSTime, RRTTime]);
set(gca, 'XTickLabel', {'A*', 'BFS', 'RRT'});
ylabel('Seconds'); title('Computation Time'); grid on;

%% ===================== SUMMARY ======================================
disp(' ');
disp('==========================================================');
disp('  Simulation complete!');
disp(['  Target order: Start -> T', num2str(BestOrder(1)), ...
    ' -> T', num2str(BestOrder(2)), ' -> T', num2str(BestOrder(3)), ...
    ' -> T', num2str(BestOrder(4)), ' -> Start']);
disp('  Figures: 1) Scenario  2) Animation  3) Comparison');
disp('==========================================================');
