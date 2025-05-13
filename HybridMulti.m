clc;clear;close all

% Define map dims
mapWidth = 30;
mapHeight = 30;

sensorRange = 1;
maxSteps = 500;
numRobots = 3;

% Coords for spawning bots
validSpawnCols = 9:14;

% Fill map with free space
map = ones(mapHeight, mapWidth);
% set border walls
map([1 end], :) = 0;
map(:, [1 end]) = 0;

% Draw some walls on the map
map(29:30, 15) = 0;
map(1:5, 15) = 0;
map(10:15, 15) = 0;
map(17:21, 10) = 0;
map(11, 1:10) = 0;
map(19, 1:10) = 0;
map(9:13, 10) = 0;
map(1:5, 10) = 0;
map(16, 25:30) = 0;
map(15, 15:20) = 0;
map(15:29, 20) = 0;
map(15:25, 15) = 0;

% Pick a random free tile as target
walkable = find(map == 1);
chosenWalkableIndex = walkable(randi(numel(walkable)));
[targetRow, targetCol] = ind2sub(size(map), chosenWalkableIndex);
map(targetRow, targetCol) = 0.5;

% slam starts unknown
slamMap = -1 * ones(size(map));
tileVisitCounts = zeros(size(map));
targetFound = false;
foundStep = NaN;

% spawn robots near bottom
robots = struct('row', [], 'col', [], 'lastDir', []);
availableCols = validSpawnCols;
for i = 1:numRobots
    cols = availableCols(randperm(numel(availableCols)));
    for column = cols
        if map(29, column) == 1
            robots(i).row = 29;
            robots(i).col = column;
            robots(i).lastDir = [0 -1];
            availableCols(availableCols == column) = [];
            break
        end
    end
end

% Init visualization
rgbMap = buildRGB(slamMap, map, tileVisitCounts);
[mapImg, roboImg] = initializeVisualization(rgbMap, robots);

% exploration loop until target or limit
for step = 1:maxSteps
    for i = 1:numRobots
        [slamMap, found] = sense(robots(i).row, robots(i).col, map, slamMap, sensorRange);
        if found && ~targetFound
            targetFound = true;
            foundStep = step;
        end

        [nextR, nextC, newDir] = plan(robots(i), slamMap, tileVisitCounts);
        [robots(i).row, robots(i).col, robots(i).lastDir] = move(nextR, nextC, robots(i).lastDir, newDir);
    end

    for i = 1:numRobots
        tileVisitCounts(robots(i).row, robots(i).col) = tileVisitCounts(robots(i).row, robots(i).col) + 1;
    end

    % Update display
    rgbMap = buildRGB(slamMap, map, tileVisitCounts);
    set(mapImg, 'CData', rgbMap);
    for i = 1:numRobots
        set(roboImg(i), 'XData', robots(i).col, 'YData', robots(i).row);
    end
    title(sprintf('SLAM Navigation | Step %d', step));
    drawnow

    % check if target found, print into console step and location
    if targetFound
        fprintf('Target found at step %d (location: [%d,%d])\n', foundStep, targetRow, targetCol);
        break
    end
    pause(0.05);
end

% Show how often each tile was visited
figure
imagesc(tileVisitCounts)
colorbar
title('Tile Visit Frequency')

% Victim Sensing
function [slamMap, found] = sense(row, column, trueMap, slamMap, range)
found = false;
[H, W] = size(slamMap);
for dx = -range:range
    for dy = -range:range
        x = row + dx;
        y = column + dy;
        if x >= 1 && x <= H && y >= 1 && y <= W
            slamMap(x, y) = trueMap(x, y);
            if trueMap(x, y) == 0.5
                found = true;
            end
        end
    end
end
end

function [nextR, nextC, dirVec] = plan(robot, slamMap, visits)
start = [robot.row, robot.col];
frontiers = findFrontiers(slamMap);

if isempty(frontiers)
    [nextR, nextC] = localGreedy(start, slamMap, visits);
else
    dists = sqrt((frontiers(:,1) - start(1)).^2 + (frontiers(:,2) - start(2)).^2);
    [~, chosenWalkableIndex] = min(dists);
    goal = frontiers(chosenWalkableIndex,:);
    path = astar(slamMap == 1 | slamMap == 0.5, start, goal);
    if size(path,1) >= 2
        nextR = path(2,1);
        nextC = path(2,2);
    else
        [nextR, nextC] = localGreedy(start, slamMap, visits);
    end
end

moveVector = [nextR - robot.row, nextC - robot.col];
if norm(moveVector) > 0
    dirVec = moveVector / norm(moveVector);
else
    dirVec = robot.lastDir / norm(robot.lastDir);
end 
end
% why does this prefer diagonal moves sometimes?

function [nextRow, nextCol] = localGreedy(pos, slamMap, visits)
dirs = [0 1; 1 0; 0 -1; -1 0; 1 1; 1 -1; -1 1; -1 -1];
potentialPath = [];
cost = [];
for d = 1:size(dirs,1)
    candidateRow = pos(1) + dirs(d,1);
    candidateCol = pos(2) + dirs(d,2);
    if candidateRow >= 1 && candidateRow <= size(slamMap,1) && candidateCol >= 1 && candidateCol <= size(slamMap,2)
        if slamMap(candidateRow,candidateCol) == 1
            potentialPath(end+1,:) = [candidateRow, candidateCol];
            cost(end+1) = visits(candidateRow, candidateCol);
        end
    end
end
if isempty(potentialPath)
    nextRow = pos(1);
    nextCol = pos(2);
else
    minCost = min(cost);
    minCostIndices = find(cost == minCost);
    sel = minCostIndices(randi(numel(minCostIndices)));
    nextRow = potentialPath(sel,1);
    nextCol = potentialPath(sel,2);
end
% picks lowest visit count tile
end

function frontiers = findFrontiers(slamMap)
[H, W] = size(slamMap);
frontiers = [];
for i = 2:H-1
    for j = 2:W-1
        if slamMap(i,j) == -1
            neighbors = [slamMap(i-1,j), slamMap(i+1,j), slamMap(i,j-1), slamMap(i,j+1)];
            if any(neighbors == 1)
                frontiers(end+1,:) = [i j];
            end
        end
    end
end
end

% a star pathfinder in slam map, took this mostly from https://github.com/petercorke/robotics-toolbox-matlab
% modified slightly to include multi-robots from https://github.com/MortezaHagh/MRPP-MATLAB

function path = astar(occGrid, start, goal)
[H, W] = size(occGrid);
G = inf(H, W);
F = inf(H, W);
cameFrom = cell(H, W);
openSet = false(H, W);
closedSet = false(H, W);
openSet(start(1), start(2)) = true;
G(start(1), start(2)) = 0;
F(start(1), start(2)) = norm(start - goal);

dirs = [0 1; 1 0; 0 -1; -1 0; 1 1; 1 -1; -1 1; -1 -1];
while any(openSet(:))
    layer = F;
    layer(~openSet) = inf;
    [~, chosenWalkableIndex] = min(layer(:));
    [row, column] = ind2sub([H W], chosenWalkableIndex);
    if row == goal(1) && column == goal(2)
        path = reconstruct(goal, cameFrom);
        return
    end
    openSet(row, column) = false;
    closedSet(row, column) = true;
    for d = 1:size(dirs,1)
        candidateRow = row + dirs(d,1);
        candidateCol = column + dirs(d,2);
        if candidateRow < 1 || candidateRow > H || candidateCol < 1 || candidateCol > W || closedSet(candidateRow,candidateCol) || ~occGrid(candidateRow,candidateCol)
            continue
        end
        tentative = G(row,column) + norm(dirs(d,:));
        if tentative < G(candidateRow,candidateCol)
            cameFrom{candidateRow,candidateCol} = [row,column];
            G(candidateRow,candidateCol) = tentative;
            F(candidateRow,candidateCol) = tentative + norm([candidateRow, candidateCol] - goal);
            openSet(candidateRow,candidateCol) = true;
        end
    end
end
path = [];
end

function p = reconstruct(pt, cameFrom)
p = pt;
while ~isempty(cameFrom{p(1),p(2)})
    p = [cameFrom{p(1),p(2)}; p];
end
end

% visualization
function [mapImg, roboImg] = initializeVisualization(rgbMap, robots)
figure
mapImg = imshow(rgbMap, 'InitialMagnification', 'fit');
hold on
roboImg = gobjects(1, numel(robots));
for k = 1:numel(robots)
    roboImg(k) = plot(robots(k).col, robots(k).row, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
end
title('SLAM Navigation | Step 0')
end

function [nextRow, nextCol, lastDir] = move(nextR, nextC, ~, dirVec)
nextRow = nextR;
nextCol = nextC;
lastDir = dirVec;

end

function rgbMap = buildRGB(slamMap, trueMap, visits)
[H, W] = size(slamMap);
rgbMap = ones(H, W, 3);
% unknown shown grey
unk = slamMap == -1;
for colour = 1:3
    layer = rgbMap(:,:,colour);
    layer(unk) = 0.5;
    rgbMap(:,:,colour) = layer;
end
% walls should be Black
wallMask = slamMap == 0;
for colour = 1:3
    layer = rgbMap(:,:,colour);
    layer(wallMask) = 0;
    rgbMap(:,:,colour) = layer;
end
% target in pink/purple maybe?
targetMask = (trueMap == 0.5 & slamMap == 0.5);
rgbMap(:,:,1) = rgbMap(:,:,1) .* (~targetMask) + 0.5 * targetMask;
rgbMap(:,:,2) = rgbMap(:,:,2) .* (~targetMask);
rgbMap(:,:,3) = rgbMap(:,:,3) .* (~targetMask) + 0.5 * targetMask;
% visits become more green
maxV = 20;
gCh = min(1, visits ./ maxV);
fMask = slamMap == 1;
rgbMap(:,:,1) = rgbMap(:,:,1) .* (~fMask) + (1 - gCh) .* fMask;
rgbMap(:,:,2) = rgbMap(:,:,2) .* (~fMask) + fMask;
rgbMap(:,:,3) = rgbMap(:,:,3) .* (~fMask) + (1 - gCh) .* fMask;
end
