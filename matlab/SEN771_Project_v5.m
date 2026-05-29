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
%    1. A* Search (workshop algorithm)
%    2. BFS - Breadth-First Search (workshop algorithm)
%    3. RRT - Rapidly-exploring Random Tree (new #1 - sampling-based)
%    4. Cluster-Based Shortest Path (new #2 - inspired by Duan et al. 2025
%       Tsinghua University, arXiv:2504.17033)
%    5. Cluster-Guided A* (new #3 - HYBRID: clusters identify influential
%       waypoints, A* plans between them. Original innovation combining
%       Tsinghua clustering with A* search)
%    6. Frontier-Reduction SSSP (new #4 - more faithful implementation of
%       Duan et al. 2025. Uses recursive frontier reduction with pivot
%       finding and bounded Bellman-Ford relaxation to bypass sorting)
%
%  Key Features:
%    - Random obstacle generation each run (user-adjustable size)
%    - Grid meshing with inflated occupied cell detection
%    - TSP brute-force target sequence optimisation
%    - Five path-finding algorithms with animated comparison
%    - Sixth algorithm: faithful Tsinghua frontier-reduction approach
%    - Fair Euclidean distance metric for all algorithms
%    - Performance analysis (distance, nodes, time)
% ========================================================================

%% Initialisation
close all; clc; clear;
disp('==========================================================');
disp('  SEN771 - Autonomous Mobile Robot Trajectory Planning');
disp('  Student: Dexter Leong | ID: s223026243');
disp('==========================================================');
disp(' ');

%% ===================== USER PARAMETERS ==============================
% Uncomment for manual input: ObsAvgSize = input('Enter avg obstacle size (5-15): ');
ObsAvgSize = 10;
MeshSize = 1;
NumObstacles = 15;
NumTargets = 4;
AnimSpeed = 8;
% Uncomment for reproducibility: rng(42);

%% ===================== FIELD & MESH =================================
disp('[1/9] Generating mesh...');
FieldLength=120; FieldWidth=90;
FieldX=0:MeshSize:FieldLength; FieldY=0:MeshSize:FieldWidth;
CellsX=FieldX(1:end-1)+MeshSize/2; CellsY=FieldY(1:end-1)+MeshSize/2;
NumCellsX=length(CellsX); NumCellsY=length(CellsY);
disp(['   Grid: ',num2str(NumCellsX),'x',num2str(NumCellsY),' = ',...
    num2str(NumCellsX*NumCellsY),' cells']);

%% ===================== OBSTACLES ====================================
disp('[2/9] Generating obstacles...');
ObstacleMap=zeros(NumCellsX,NumCellsY);
ObsRect=zeros(NumObstacles,4);
for obs=1:NumObstacles
    w=ObsAvgSize*(0.5+rand()); h=ObsAvgSize*(0.5+rand());
    ox=w/2+rand()*(FieldLength-w); oy=h/2+rand()*(FieldWidth-h);
    ObsRect(obs,:)=[ox-w/2,oy-h/2,w,h];
    xMn=ox-w/2; xMx=ox+w/2; yMn=oy-h/2; yMx=oy+h/2;
    iMn=max(1,floor(xMn/MeshSize)); iMx=min(NumCellsX,ceil(xMx/MeshSize));
    jMn=max(1,floor(yMn/MeshSize)); jMx=min(NumCellsY,ceil(yMx/MeshSize));
    for i=iMn:iMx; for j=jMn:jMx
        if CellsX(i)>=xMn&&CellsX(i)<=xMx&&CellsY(j)>=yMn&&CellsY(j)<=yMx
            ObstacleMap(i,j)=1;
        end
    end; end
end
InflatedMap=ObstacleMap;
for i=1:NumCellsX; for j=1:NumCellsY
    if ObstacleMap(i,j)==1
        for di=-1:1; for dj=-1:1
            ni=i+di; nj=j+dj;
            if ni>=1&&ni<=NumCellsX&&nj>=1&&nj<=NumCellsY; InflatedMap(ni,nj)=1; end
        end; end
    end
end; end
disp(['   Obstacles: ',num2str(NumObstacles),' | Inflated cells: ',num2str(sum(InflatedMap(:)))]);

%% ===================== ROBOT START ==================================
RobotStart=[1,round(NumCellsY/2)]';
while InflatedMap(RobotStart(1),RobotStart(2))==1
    RobotStart(2)=RobotStart(2)+1;
    if RobotStart(2)>NumCellsY; RobotStart(2)=1; end
end

%% ===================== TARGETS ======================================
disp('[3/9] Placing targets...');
TargetCells=zeros(2,NumTargets);
for t=1:NumTargets
    placed=false;
    while ~placed
        tx=randi([round(NumCellsX*0.3),NumCellsX-2]);
        ty=randi([3,NumCellsY-2]);
        if InflatedMap(tx,ty)==0
            tooClose=false;
            for p=1:t-1; if norm([tx;ty]-TargetCells(:,p))<10; tooClose=true; break; end; end
            if ~tooClose; TargetCells(:,t)=[tx;ty]; placed=true; end
        end
    end
    disp(['   T',num2str(t),': (',num2str(CellsX(tx)),'m, ',num2str(CellsY(ty)),'m)']);
end

%% ===================== SCENARIO PLOT ================================
disp('[4/9] Drawing scenario...');
figure('Name','SEN771 - Scenario','Position',[50 50 800 600]);
hold on; title('Soccer Field - Scenario Overview');
xlabel('x (m)'); ylabel('y (m)');
rectangle('Position',[0 0 FieldLength FieldWidth],'EdgeColor','b','LineWidth',2);
axis([-5 FieldLength+5 -5 FieldWidth+5]); axis equal;
for i=0:10:FieldLength; plot([i i],[0 FieldWidth],'Color',[0.9 0.9 0.9]); end
for j=0:10:FieldWidth; plot([0 FieldLength],[j j],'Color',[0.9 0.9 0.9]); end
for obs=1:NumObstacles; rectangle('Position',ObsRect(obs,:),'FaceColor',[1 0 0 0.7],'EdgeColor','r'); end
for i=1:NumCellsX; for j=1:NumCellsY
    if ObstacleMap(i,j)==1; plot(CellsX(i),CellsY(j),'r.','MarkerSize',3); end
end; end
for t=1:NumTargets
    plot(CellsX(TargetCells(1,t)),CellsY(TargetCells(2,t)),'go','MarkerSize',12,'LineWidth',2,'MarkerFaceColor','g');
    text(CellsX(TargetCells(1,t))+2,CellsY(TargetCells(2,t))+2,['T',num2str(t)],'FontSize',10,'FontWeight','bold','Color',[0 0.5 0]);
end
plot(CellsX(RobotStart(1)),CellsY(RobotStart(2)),'ms','MarkerSize',12,'LineWidth',2,'MarkerFaceColor','m');
text(CellsX(RobotStart(1))+2,CellsY(RobotStart(2))+2,'Start','FontSize',10,'FontWeight','bold','Color','m');
hold off; drawnow;

%% ===================== TSP OPTIMISATION ==============================
disp('[5/9] Optimising target order (TSP)...');
Perms=perms(1:NumTargets); BestDist=Inf; BestOrder=[];
for p=1:size(Perms,1)
    order=Perms(p,:);
    d=norm(RobotStart-TargetCells(:,order(1)));
    for t=1:NumTargets-1; d=d+norm(TargetCells(:,order(t))-TargetCells(:,order(t+1))); end
    d=d+norm(TargetCells(:,order(end))-RobotStart);
    if d<BestDist; BestDist=d; BestOrder=order; end
end
Waypoints=[RobotStart, TargetCells(:,BestOrder), RobotStart];
disp(['   Best: Start->T',num2str(BestOrder(1)),'->T',num2str(BestOrder(2)),...
    '->T',num2str(BestOrder(3)),'->T',num2str(BestOrder(4)),'->Start']);

%% ===================== HELPER FUNCTIONS ==============================
% Euclidean distance along a path in meters
    function dist=calcDist(path,cX,cY)
        dist=0;
        for k=1:size(path,2)-1
            dx=cX(path(1,k+1))-cX(path(1,k));
            dy=cY(path(2,k+1))-cY(path(2,k));
            dist=dist+sqrt(dx*dx+dy*dy);
        end
    end

% Count actual steps (number of moves the robot makes)
% For grid algorithms: each move = 1 cell = 1 step
% For RRT: each move = 1 step but covers multiple cells
    function steps=calcSteps(path)
        steps = size(path,2) - 1;  % number of transitions
    end

% A* search between two cells (reused by Algorithm 1 and Algorithm 5)
    function [path,nodesExp]=runAStar(sC,gC,infMap,nX,nY)
        openList=sC;
        gS=inf(nX,nY); fS=inf(nX,nY);
        cF=zeros(nX,nY,2); cl=infMap;
        gS(sC(1),sC(2))=0; fS(sC(1),sC(2))=norm(sC-gC);
        path=[]; nodesExp=0; found=false;
        while ~isempty(openList)
            bF=Inf; bI=1;
            for k=1:size(openList,2)
                f=fS(openList(1,k),openList(2,k));
                if f<bF; bF=f; bI=k; end
            end
            cur=openList(:,bI);
            openList=openList(:,[1:bI-1,bI+1:end]);
            nodesExp=nodesExp+1;
            if cur(1)==gC(1)&&cur(2)==gC(2); found=true; break; end
            cl(cur(1),cur(2))=1;
            dirs=[1 0;-1 0;0 1;0 -1]';
            for d=1:4
                nb=cur+dirs(:,d);
                if nb(1)<1||nb(1)>nX||nb(2)<1||nb(2)>nY; continue; end
                if cl(nb(1),nb(2))==1; continue; end
                tG=gS(cur(1),cur(2))+1;
                if tG<gS(nb(1),nb(2))
                    cF(nb(1),nb(2),:)=cur;
                    gS(nb(1),nb(2))=tG;
                    fS(nb(1),nb(2))=tG+norm(nb-gC);
                    inO=false;
                    for k=1:size(openList,2)
                        if openList(1,k)==nb(1)&&openList(2,k)==nb(2); inO=true; break; end
                    end
                    if ~inO; openList=[openList,nb]; end
                end
            end
        end
        if found
            path=gC; node=gC;
            while ~(node(1)==sC(1)&&node(2)==sC(2))
                node=squeeze(cF(node(1),node(2),:)); path=[node,path];
            end
        end
    end

%% ===================== ALGORITHM 1: A* ==============================
disp(' ');
disp('[6/9] Running all 5 path-finding algorithms...');
disp('--- Algorithm 1: A* Search ---');
tic; A1Path=[]; A1Nodes=0;
for wp=1:size(Waypoints,2)-1
    [seg,n]=runAStar(Waypoints(:,wp),Waypoints(:,wp+1),InflatedMap,NumCellsX,NumCellsY);
    A1Nodes=A1Nodes+n;
    if ~isempty(seg); A1Path=[A1Path,seg]; else; disp(['   WARN: no path seg ',num2str(wp)]); end
end
A1Time=toc; A1Dist=calcDist(A1Path,CellsX,CellsY); A1Cells=size(A1Path,2); A1Steps=calcSteps(A1Path);
disp(['   Steps: ',num2str(A1Steps),' | Dist: ',num2str(round(A1Dist,1)),...
    'm | Nodes: ',num2str(A1Nodes),' | Time: ',num2str(round(A1Time,3)),'s']);

%% ===================== ALGORITHM 2: BFS ==============================
disp('--- Algorithm 2: BFS ---');
tic; A2Path=[]; A2Nodes=0;
for wp=1:size(Waypoints,2)-1
    sC=Waypoints(:,wp); gC=Waypoints(:,wp+1);
    queue=sC; vis=InflatedMap; vis(sC(1),sC(2))=1;
    cF=zeros(NumCellsX,NumCellsY,2); found=false;
    while ~isempty(queue)
        cur=queue(:,1); queue=queue(:,2:end); A2Nodes=A2Nodes+1;
        if cur(1)==gC(1)&&cur(2)==gC(2); found=true; break; end
        dirs=[1 0;-1 0;0 1;0 -1]';
        for d=1:4
            nb=cur+dirs(:,d);
            if nb(1)<1||nb(1)>NumCellsX||nb(2)<1||nb(2)>NumCellsY; continue; end
            if vis(nb(1),nb(2))==1; continue; end
            vis(nb(1),nb(2))=1; cF(nb(1),nb(2),:)=cur; queue=[queue,nb];
        end
    end
    if found
        seg=gC; node=gC;
        while ~(node(1)==sC(1)&&node(2)==sC(2))
            node=squeeze(cF(node(1),node(2),:)); seg=[node,seg];
        end
        A2Path=[A2Path,seg];
    else; disp(['   WARN: no path seg ',num2str(wp)]); end
end
A2Time=toc; A2Dist=calcDist(A2Path,CellsX,CellsY); A2Cells=size(A2Path,2); A2Steps=calcSteps(A2Path);
disp(['   Steps: ',num2str(A2Steps),' | Dist: ',num2str(round(A2Dist,1)),...
    'm | Nodes: ',num2str(A2Nodes),' | Time: ',num2str(round(A2Time,3)),'s']);

%% ===================== ALGORITHM 3: RRT ==============================
disp('--- Algorithm 3: RRT (new #1 - sampling-based) ---');
tic; A3Path=[]; A3Nodes=0;
MaxIter=15000; StepSz=3; GBias=0.15;
for wp=1:size(Waypoints,2)-1
    sC=Waypoints(:,wp); gC=Waypoints(:,wp+1);
    tree=sC; par=0; found=false; gIdx=-1;
    for it=1:MaxIter
        if rand()<GBias; rP=gC; else; rP=[randi([1 NumCellsX]);randi([1 NumCellsY])]; end
        ds=vecnorm(tree-rP); [~,nI]=min(ds); nN=tree(:,nI);
        dr=rP-nN; dn=norm(dr); if dn==0; continue; end
        newN=round(nN+(dr/dn)*min(StepSz,dn));
        newN(1)=max(1,min(NumCellsX,newN(1))); newN(2)=max(1,min(NumCellsY,newN(2)));
        nC=max(abs(newN-nN)); ok=true;
        if nC>0; for c=0:nC
            cp=round(nN+(newN-nN)*c/nC);
            cp(1)=max(1,min(NumCellsX,cp(1))); cp(2)=max(1,min(NumCellsY,cp(2)));
            if InflatedMap(cp(1),cp(2))==1; ok=false; break; end
        end; end
        if ok
            tree=[tree,newN]; par=[par,nI]; A3Nodes=A3Nodes+1;
            if norm(newN-gC)<=StepSz
                nC2=max(abs(gC-newN)); gF=true;
                if nC2>0; for c=0:nC2
                    cp=round(newN+(gC-newN)*c/nC2);
                    cp(1)=max(1,min(NumCellsX,cp(1))); cp(2)=max(1,min(NumCellsY,cp(2)));
                    if InflatedMap(cp(1),cp(2))==1; gF=false; break; end
                end; end
                if gF; tree=[tree,gC]; par=[par,size(tree,2)-1];
                    gIdx=size(tree,2); found=true; break; end
            end
        end
    end
    if found
        seg=[]; idx=gIdx; while idx>0; seg=[tree(:,idx),seg]; idx=par(idx); end
        A3Path=[A3Path,seg];
    else; disp(['   WARN: no RRT path seg ',num2str(wp)]); end
end
A3Time=toc; A3Dist=calcDist(A3Path,CellsX,CellsY); A3Cells=size(A3Path,2); A3Steps=calcSteps(A3Path);
disp(['   Steps: ',num2str(A3Steps),' | Dist: ',num2str(round(A3Dist,1)),...
    'm | Nodes: ',num2str(A3Nodes),' | Time: ',num2str(round(A3Time,3)),'s']);

%% ===================== ALGORITHM 4: CLUSTER-BASED ====================
% Standalone cluster algorithm inspired by Duan et al. 2025
% Processes influential nodes per cluster without priority queue sorting
disp('--- Algorithm 4: Cluster-Based (Tsinghua-inspired, new #2) ---');
tic; A4Path=[]; A4Nodes=0;
ClustSz=10; BF_steps=5;
for wp=1:size(Waypoints,2)-1
    sC=Waypoints(:,wp); gC=Waypoints(:,wp+1);
    distE=inf(NumCellsX,NumCellsY); distE(sC(1),sC(2))=0;
    cF=zeros(NumCellsX,NumCellsY,2); done=InflatedMap; found=false;
    maxWaves=NumCellsX+NumCellsY;
    for wave=1:maxWaves
        if found; break; end
        % Find frontier cells (finite dist, not done)
        frontier=[];
        for i=1:NumCellsX; for j=1:NumCellsY
            if distE(i,j)<Inf && done(i,j)==0; frontier=[frontier,[i;j]]; end
        end; end
        if isempty(frontier); break; end
        % Group into clusters, find influential node per cluster
        cIDs=[ceil(frontier(1,:)/ClustSz); ceil(frontier(2,:)/ClustSz)];
        uC=unique(cIDs','rows')';
        infNodes=[];
        for ci=1:size(uC,2)
            cx=uC(1,ci); cy=uC(2,ci);
            iMn=(cx-1)*ClustSz+1; iMx=min(cx*ClustSz,NumCellsX);
            jMn=(cy-1)*ClustSz+1; jMx=min(cy*ClustSz,NumCellsY);
            bD=Inf; bC=[];
            for i=iMn:iMx; for j=jMn:jMx
                if distE(i,j)<Inf && done(i,j)==0
                    nR=0; dirs=[1 0;-1 0;0 1;0 -1]';
                    for d=1:4; nb=[i;j]+dirs(:,d);
                        if nb(1)>=1&&nb(1)<=NumCellsX&&nb(2)>=1&&nb(2)<=NumCellsY
                            if done(nb(1),nb(2))==0; nR=nR+1; end
                        end
                    end
                    sc=distE(i,j)-nR*0.1;
                    if sc<bD; bD=sc; bC=[i;j]; end
                end
            end; end
            if ~isempty(bC); infNodes=[infNodes,bC]; end
        end
        % Limited Bellman-Ford from influential nodes
        active=infNodes;
        for bf=1:BF_steps
            nxt=[];
            for a=1:size(active,2)
                ci=active(1,a); cj=active(2,a);
                if done(ci,cj)==0; done(ci,cj)=1; A4Nodes=A4Nodes+1; end
                dirs=[1 0;-1 0;0 1;0 -1]';
                for d=1:4
                    nb=[ci;cj]+dirs(:,d);
                    if nb(1)<1||nb(1)>NumCellsX||nb(2)<1||nb(2)>NumCellsY; continue; end
                    if InflatedMap(nb(1),nb(2))==1; continue; end
                    nD=distE(ci,cj)+1;
                    if nD<distE(nb(1),nb(2))
                        distE(nb(1),nb(2))=nD; cF(nb(1),nb(2),:)=[ci;cj];
                        if done(nb(1),nb(2))==0; nxt=[nxt,nb]; end
                    end
                end
                if ci==gC(1)&&cj==gC(2); found=true; break; end
            end
            if found; break; end
            active=nxt; if isempty(active); break; end
        end
    end
    if found
        seg=gC; node=gC;
        while ~(node(1)==sC(1)&&node(2)==sC(2))
            prev=squeeze(cF(node(1),node(2),:));
            if prev(1)==0&&prev(2)==0; disp('   WARN: reconstruction fail'); break; end
            seg=[prev,seg]; node=prev;
        end
        A4Path=[A4Path,seg];
    else; disp(['   WARN: no Cluster path seg ',num2str(wp)]); end
end
A4Time=toc; A4Dist=calcDist(A4Path,CellsX,CellsY); A4Cells=size(A4Path,2); A4Steps=calcSteps(A4Path);
disp(['   Steps: ',num2str(A4Steps),' | Dist: ',num2str(round(A4Dist,1)),...
    'm | Nodes: ',num2str(A4Nodes),' | Time: ',num2str(round(A4Time,3)),'s']);

%% ===================== ALGORITHM 5: CLUSTER-GUIDED A* ================
% HYBRID innovation: Cluster analysis identifies influential waypoints
% between start and goal, then A* plans short hops between them.
% This reduces A*'s search space per segment while the clustering
% provides global strategic direction.
%
% How it works:
%   1. Divide field into clusters (10x10 cells)
%   2. For each segment (start->target), find clusters along the
%      straight-line corridor between them
%   3. In each corridor cluster, find the best free cell (closest to
%      the line connecting start and goal) = influential waypoint
%   4. Order waypoints by distance from start
%   5. Run A* between consecutive waypoints (short hops = fast A*)
%   6. Concatenate all sub-paths
disp('--- Algorithm 5: Cluster-Guided A* (HYBRID innovation, new #3) ---');
tic; A5Path=[]; A5Nodes=0;
CorridorWidth = 15;  % cells either side of the direct line to search

for wp=1:size(Waypoints,2)-1
    sC=Waypoints(:,wp); gC=Waypoints(:,wp+1);
    
    % Step 1: Find clusters along the corridor from sC to gC
    % The corridor is a band of cells within CorridorWidth of the
    % straight line from start to goal
    lineDir = gC - sC;
    lineLen = norm(lineDir);
    if lineLen == 0; continue; end
    lineUnit = lineDir / lineLen;
    lineNorm = [-lineUnit(2); lineUnit(1)];  % perpendicular
    
    % Step 2: Sample influential waypoints along the corridor
    % Divide the line into segments based on cluster size
    numSegs = max(2, round(lineLen / ClustSz));
    subWaypoints = sC;  % start with the start cell
    
    for s = 1:numSegs-1
        % Point along the direct line at fraction s/numSegs
        frac = s / numSegs;
        linePoint = round(sC + lineDir * frac);
        linePoint(1) = max(1, min(NumCellsX, linePoint(1)));
        linePoint(2) = max(1, min(NumCellsY, linePoint(2)));
        
        % Search nearby for the best free cell (influential waypoint)
        bestWP = [];
        bestScore = Inf;
        searchR = round(CorridorWidth/2);
        
        for di = -searchR:searchR
            for dj = -searchR:searchR
                ci = linePoint(1) + di;
                cj = linePoint(2) + dj;
                if ci<1||ci>NumCellsX||cj<1||cj>NumCellsY; continue; end
                if InflatedMap(ci,cj)==1; continue; end
                
                % Score: distance from the direct line (prefer cells near it)
                % plus small penalty for distance from the line point
                vec = [ci;cj] - sC;
                perpDist = abs(vec(1)*lineNorm(1) + vec(2)*lineNorm(2));
                alongDist = abs(norm(vec) - frac*lineLen);
                score = perpDist + alongDist * 0.5;
                
                if score < bestScore
                    bestScore = score;
                    bestWP = [ci;cj];
                end
            end
        end
        
        if ~isempty(bestWP)
            % Only add if it's not too close to the last waypoint
            if norm(bestWP - subWaypoints(:,end)) > 3
                subWaypoints = [subWaypoints, bestWP];
            end
        end
    end
    
    subWaypoints = [subWaypoints, gC];  % end with the goal
    
    % Step 3: Run A* between consecutive sub-waypoints (short hops)
    for sw = 1:size(subWaypoints,2)-1
        [seg, n] = runAStar(subWaypoints(:,sw), subWaypoints(:,sw+1), ...
            InflatedMap, NumCellsX, NumCellsY);
        A5Nodes = A5Nodes + n;
        if ~isempty(seg)
            % Avoid duplicating the connection cell
            if ~isempty(A5Path) && sw > 1
                seg = seg(:, 2:end);  % remove first cell (already in path)
            end
            A5Path = [A5Path, seg];
        else
            % Fallback: if short hop fails, try direct A* for this segment
            disp(['   Note: short hop failed at sub-waypoint ',num2str(sw),...
                ', falling back to direct A*']);
            [seg, n] = runAStar(subWaypoints(:,1), subWaypoints(:,end), ...
                InflatedMap, NumCellsX, NumCellsY);
            A5Nodes = A5Nodes + n;
            if ~isempty(seg); A5Path = [A5Path, seg]; end
            break;
        end
    end
end
A5Time=toc; A5Dist=calcDist(A5Path,CellsX,CellsY); A5Cells=size(A5Path,2); A5Steps=calcSteps(A5Path);
disp(['   Steps: ',num2str(A5Steps),' | Dist: ',num2str(round(A5Dist,1)),...
    'm | Nodes: ',num2str(A5Nodes),' | Time: ',num2str(round(A5Time,3)),'s']);

%% ===================== ALGORITHM 6: FRONTIER-REDUCTION SSSP ==========
% More faithful implementation of Duan et al. 2025
% "Breaking the Sorting Barrier for Directed Single-Source Shortest Paths"
% Key paper concepts implemented:
%   1. Frontier = set of unfinished nodes with finite distance estimates
%   2. Pivot finding = identify nodes that appear on many shortest paths
%      (we approximate this by finding nodes with best distance-to-reach ratio)
%   3. Bounded Bellman-Ford = relax edges from pivots for k iterations only
%   4. Frontier reduction = after each round, some frontier nodes get
%      finalised, shrinking the frontier without ever sorting it globally
%   5. Recursive subdivision = split remaining frontier into smaller
%      subproblems (we approximate with distance-band subdivision)
%
% The key difference from Algorithm 4:
%   - Alg 4 uses fixed clusters based on spatial position
%   - Alg 6 uses dynamic frontiers based on distance estimates, with
%     recursive subdivision that shrinks the active set each round
%     (closer to the paper's actual approach)
disp(' ');
disp('--- Algorithm 6: Frontier-Reduction SSSP (Tsinghua faithful, new #4) ---');
disp('   [Based on recursive frontier reduction + pivot BF relaxation]');

tic; A6Path=[]; A6Nodes=0;

for wp=1:size(Waypoints,2)-1
    sC=Waypoints(:,wp); gC=Waypoints(:,wp+1);
    
    % Distance estimates
    dist6 = inf(NumCellsX, NumCellsY);
    dist6(sC(1),sC(2)) = 0;
    cameFrom6 = zeros(NumCellsX, NumCellsY, 2);
    finalised = InflatedMap;  % obstacles are pre-finalised
    found = false;
    
    % Initial frontier: just the start node
    frontier = sC;  % columns = frontier cell indices
    
    maxRounds = 500;  % safety limit
    
    for rnd = 1:maxRounds
        if found || isempty(frontier); break; end
        
        % ============================================================
        % STEP 1: PIVOT FINDING (inspired by FindPivots in the paper)
        % Find "influential" nodes in the frontier — nodes that have
        % low distance AND high connectivity to unfinalised neighbours.
        % The paper uses shortest-path DAG centrality; we approximate
        % with: score = distance - 0.3 * (number of free neighbours)
        % Lower score = more influential = better pivot
        % ============================================================
        numPivots = max(1, round(size(frontier,2) / 3));  % ~1/3 of frontier are pivots
        pivotScores = zeros(1, size(frontier,2));
        
        for f = 1:size(frontier,2)
            fi = frontier(1,f); fj = frontier(2,f);
            nFree = 0;
            dirs=[1 0;-1 0;0 1;0 -1]';
            for d=1:4
                nb=[fi;fj]+dirs(:,d);
                if nb(1)>=1&&nb(1)<=NumCellsX&&nb(2)>=1&&nb(2)<=NumCellsY
                    if finalised(nb(1),nb(2))==0; nFree=nFree+1; end
                end
            end
            pivotScores(f) = dist6(fi,fj) - 0.3 * nFree;
        end
        
        % Select top pivots (lowest scores)
        [~, sortedIdx] = sort(pivotScores);
        numP = min(numPivots, length(sortedIdx));
        pivots = frontier(:, sortedIdx(1:numP));
        
        % ============================================================
        % STEP 2: BOUNDED BELLMAN-FORD FROM PIVOTS
        % Relax edges outward from pivots for a bounded number of steps.
        % This propagates distance info WITHOUT sorting.
        % The paper calls this the BMSSP subroutine.
        % ============================================================
        BF_bound = 8;  % bounded relaxation depth
        activeSet = pivots;
        
        for bf = 1:BF_bound
            if found; break; end
            nextActive = [];
            
            for a = 1:size(activeSet, 2)
                ai = activeSet(1,a); aj = activeSet(2,a);
                
                % Finalise this node (equivalent to "settling" in Dijkstra
                % but without global sorting — we trust the BF relaxation)
                if finalised(ai,aj)==0
                    finalised(ai,aj) = 1;
                    A6Nodes = A6Nodes + 1;
                end
                
                % Relax all 4 neighbours
                dirs=[1 0;-1 0;0 1;0 -1]';
                for d=1:4
                    nb=[ai;aj]+dirs(:,d);
                    if nb(1)<1||nb(1)>NumCellsX||nb(2)<1||nb(2)>NumCellsY; continue; end
                    if InflatedMap(nb(1),nb(2))==1; continue; end
                    
                    newDist = dist6(ai,aj) + 1;
                    if newDist < dist6(nb(1),nb(2))
                        dist6(nb(1),nb(2)) = newDist;
                        cameFrom6(nb(1),nb(2),:) = [ai;aj];
                        if finalised(nb(1),nb(2))==0
                            nextActive = [nextActive, nb];
                        end
                    end
                end
                
                % Goal check
                if ai==gC(1) && aj==gC(2); found=true; break; end
            end
            
            if found; break; end
            activeSet = nextActive;
            if isempty(activeSet); break; end
        end
        
        % ============================================================
        % STEP 3: FRONTIER REDUCTION (key paper concept)
        % After BF relaxation, rebuild the frontier from scratch.
        % Only include nodes that: have finite distance, are not finalised.
        % This is the "reduction" — the frontier shrinks each round
        % because the BF step finalised some nodes.
        %
        % The paper additionally subdivides the frontier by distance bands
        % (recursive subdivision). We approximate this by only keeping
        % nodes within a reasonable distance band of the current minimum.
        % ============================================================
        newFrontier = [];
        minFrontierDist = Inf;
        
        % First pass: find minimum unfinalised distance
        for i=1:NumCellsX
            for j=1:NumCellsY
                if dist6(i,j)<Inf && finalised(i,j)==0
                    if dist6(i,j) < minFrontierDist
                        minFrontierDist = dist6(i,j);
                    end
                end
            end
        end
        
        % Second pass: only keep frontier nodes within a distance band
        % This is the subdivision — process nearby nodes first, defer far ones
        distBand = max(20, minFrontierDist * 0.5);  % adaptive band width
        
        for i=1:NumCellsX
            for j=1:NumCellsY
                if dist6(i,j)<Inf && finalised(i,j)==0
                    if dist6(i,j) <= minFrontierDist + distBand
                        newFrontier = [newFrontier, [i;j]];
                    end
                end
            end
        end
        
        frontier = newFrontier;
    end
    
    % Reconstruct path
    if found
        seg=gC; node=gC;
        while ~(node(1)==sC(1) && node(2)==sC(2))
            prev=squeeze(cameFrom6(node(1),node(2),:));
            if prev(1)==0&&prev(2)==0
                disp('   WARN: path reconstruction failed');
                break;
            end
            seg=[prev,seg]; node=prev;
        end
        A6Path=[A6Path,seg];
    else
        disp(['   WARN: no Frontier-Reduction path for segment ',num2str(wp)]);
    end
end
A6Time=toc; A6Dist=calcDist(A6Path,CellsX,CellsY); A6Cells=size(A6Path,2); A6Steps=calcSteps(A6Path);
disp(['   Steps: ',num2str(A6Steps),' | Dist: ',num2str(round(A6Dist,1)),...
    'm | Nodes: ',num2str(A6Nodes),' | Time: ',num2str(round(A6Time,3)),'s']);

%% ===================== ANIMATED MOVEMENT =============================
disp(' ');
disp('[8/10] Animating movement...');
algNames={'A*','BFS','RRT','Cluster','Clust-A*','Frontier-Red'};
algPaths={A1Path,A2Path,A3Path,A4Path,A5Path,A6Path};
algClrs={'b',[0 0.7 0],[0.8 0.4 0],[0.6 0 0.6],[0 0.5 0.8],[0.8 0 0]};
numAlgs=6;

figure('Name','SEN771 - Animation','Position',[20 20 1800 350]);
for alg=1:numAlgs
    subplot(2,3,alg); hold on;
    title([algNames{alg},' - Movement'],'FontSize',9);
    xlabel('x (m)'); ylabel('y (m)');
    rectangle('Position',[0 0 FieldLength FieldWidth],'EdgeColor','b','LineWidth',1.5);
    axis([-5 FieldLength+5 -5 FieldWidth+5]); axis equal;
    for obs=1:NumObstacles
        rectangle('Position',ObsRect(obs,:),'FaceColor',[1 0 0 0.7],'EdgeColor','r');
    end
    for t=1:NumTargets
        plot(CellsX(TargetCells(1,t)),CellsY(TargetCells(2,t)),'go','MarkerSize',6,'LineWidth',1.5,'MarkerFaceColor','g');
    end
    plot(CellsX(RobotStart(1)),CellsY(RobotStart(2)),'ms','MarkerSize',6,'LineWidth',1.5,'MarkerFaceColor','m');
end
drawnow;

maxSteps=0;
for alg=1:numAlgs; maxSteps=max(maxSteps,size(algPaths{alg},2)); end
rH=gobjects(1,numAlgs); tH=gobjects(1,numAlgs);
tX=cell(1,numAlgs); tY=cell(1,numAlgs);
for alg=1:numAlgs; tX{alg}=[]; tY{alg}=[]; end
for alg=1:numAlgs
    if ~isempty(algPaths{alg})
        subplot(2,3,alg);
        rH(alg)=plot(CellsX(algPaths{alg}(1,1)),CellsY(algPaths{alg}(2,1)),'ms','MarkerSize',8,'LineWidth',1.5,'MarkerFaceColor','m');
    end
end
step=1;
while step<=maxSteps
    for alg=1:numAlgs
        pth=algPaths{alg}; if isempty(pth); continue; end
        s=min(step,size(pth,2));
        px=CellsX(pth(1,s)); py=CellsY(pth(2,s));
        tX{alg}=[tX{alg},px]; tY{alg}=[tY{alg},py];
        subplot(2,3,alg);
        if isvalid(tH(alg)); delete(tH(alg)); end
        tH(alg)=plot(tX{alg},tY{alg},'-','Color',algClrs{alg},'LineWidth',1.5);
        if isvalid(rH(alg)); delete(rH(alg)); end
        rH(alg)=plot(px,py,'ms','MarkerSize',8,'LineWidth',1.5,'MarkerFaceColor','m');
    end
    drawnow; pause(0.003); step=step+AnimSpeed;
end
for alg=1:numAlgs
    pth=algPaths{alg}; if isempty(pth); continue; end
    subplot(2,3,alg);
    plot(CellsX(pth(1,:)),CellsY(pth(2,:)),'-','Color',algClrs{alg},'LineWidth',2);
end
drawnow;

%% ===================== PERFORMANCE COMPARISON ========================
disp(' ');
disp('[9/10] Performance comparison...');
disp(' ');
disp('============================== PERFORMANCE COMPARISON ==============================');
fprintf('%-24s %-10s %-14s %-14s %-14s %-10s\n','Algorithm','Steps','Dist (m)','Nodes','Time (s)','Dist/Step');
disp('------------------------------------------------------------------------------------');
fprintf('%-24s %-10d %-14.1f %-14d %-14.3f %-10.2f\n','1. A*',A1Steps,A1Dist,A1Nodes,A1Time,A1Dist/max(1,A1Steps));
fprintf('%-24s %-10d %-14.1f %-14d %-14.3f %-10.2f\n','2. BFS',A2Steps,A2Dist,A2Nodes,A2Time,A2Dist/max(1,A2Steps));
fprintf('%-24s %-10d %-14.1f %-14d %-14.3f %-10.2f\n','3. RRT',A3Steps,A3Dist,A3Nodes,A3Time,A3Dist/max(1,A3Steps));
fprintf('%-24s %-10d %-14.1f %-14d %-14.3f %-10.2f\n','4. Cluster (Tsinghua)',A4Steps,A4Dist,A4Nodes,A4Time,A4Dist/max(1,A4Steps));
fprintf('%-24s %-10d %-14.1f %-14d %-14.3f %-10.2f\n','5. Cluster-Guided A*',A5Steps,A5Dist,A5Nodes,A5Time,A5Dist/max(1,A5Steps));
fprintf('%-24s %-10d %-14.1f %-14d %-14.3f %-10.2f\n','6. Frontier-Reduction',A6Steps,A6Dist,A6Nodes,A6Time,A6Dist/max(1,A6Steps));
disp('------------------------------------------------------------------------------------');
disp(' ');
disp('  Steps    = number of moves the robot makes');
disp('  Dist     = total Euclidean distance travelled (meters)');
disp('  Dist/Step = average distance per move (1.0 for grid, >1 for RRT)');

%% ===================== BAR CHARTS ====================================
disp('[10/10] Plotting comparison charts...');
figure('Name','SEN771 - Performance','Position',[80 80 1600 400]);
names={'A*','BFS','RRT','Cluster','Clust-A*','Front-Red'};

subplot(1,4,1);
bar([A1Steps,A2Steps,A3Steps,A4Steps,A5Steps,A6Steps]);
set(gca,'XTickLabel',names,'FontSize',7); ylabel('Steps'); title('Number of Steps'); grid on;

subplot(1,4,2);
bar([A1Dist,A2Dist,A3Dist,A4Dist,A5Dist,A6Dist]);
set(gca,'XTickLabel',names,'FontSize',7); ylabel('Meters'); title('Euclidean Distance'); grid on;

subplot(1,4,3);
bar([A1Nodes,A2Nodes,A3Nodes,A4Nodes,A5Nodes,A6Nodes]);
set(gca,'XTickLabel',names,'FontSize',7); ylabel('Nodes'); title('Nodes Explored'); grid on;

subplot(1,4,4);
bar([A1Time,A2Time,A3Time,A4Time,A5Time,A6Time]);
set(gca,'XTickLabel',names,'FontSize',7); ylabel('Seconds'); title('Computation Time'); grid on;

%% ===================== SUMMARY ======================================
disp(' ');
disp('==========================================================');
disp('  Simulation complete!');
disp(['  Target order: Start->T',num2str(BestOrder(1)),'->T',...
    num2str(BestOrder(2)),'->T',num2str(BestOrder(3)),'->T',...
    num2str(BestOrder(4)),'->Start']);
disp(' ');
disp('  Algorithms:');
disp('    1. A* Search (workshop)');
disp('    2. BFS (workshop)');
disp('    3. RRT (new - sampling-based)');
disp('    4. Cluster-Based (new - Tsinghua-inspired)');
disp('    5. Cluster-Guided A* (new - hybrid innovation)');
disp('    6. Frontier-Reduction SSSP (new - faithful Tsinghua)');
disp(' ');
disp('  Figures: 1) Scenario  2) Animation  3) Comparison');
disp('==========================================================');
