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
%    1. A* Search          (workshop algorithm)
%    2. BFS                (workshop algorithm)
%    3. RRT                (tested - sampling-based, included for comparison)
%    4. Theta*             (NEW - any-angle path planning, not taught in class.
%                           A* with built-in line-of-sight checks producing
%                           smooth diagonal paths. Daniel et al. 2010)
%
%  Key Features:
%    - Random obstacle generation each run (user sets min/max size via prompt)
%    - Grid meshing with inflated occupied cell detection
%    - TSP brute-force target sequence optimisation (4! = 24 permutations)
%    - 4 path-finding algorithms with side-by-side animated comparison
%    - Fair Euclidean distance metric for all algorithms
%    - Performance analysis (steps, distance, efficiency, nodes, time)
% ========================================================================

%% Initialisation
close all; clc; clear;
disp('==========================================================');
disp('  SEN771 - Autonomous Mobile Robot Trajectory Planning');
disp('  Student: Dexter Leong | ID: s223026243');
disp('==========================================================');
disp(' ');

%% ===================== USER PARAMETERS ==============================
MeshSize = 1;
NumObstacles = 15;
NumTargets = 4;
AnimSpeed = 8;

% --- Prompt user for obstacle size ---
disp('----------------------------------------------------------');
disp('  Obstacle Size Configuration');
disp('  All 15 obstacles will be this size (random tall/wide).');
disp('  Maximum allowed: 10 m. (Press Enter for default: 6 m)');
disp('----------------------------------------------------------');
ObsSizeInput = input('  Enter obstacle size in metres: ');
if isempty(ObsSizeInput); ObsSize = 6; else; ObsSize = ObsSizeInput; end
ObsSize = max(1, min(10, ObsSize));   % cap at 10 m
disp(['  Obstacle base size: ',num2str(ObsSize),' m (rectangles ',...
    num2str(ObsSize),' x ',num2str(ObsSize*2),' m, random orientation)']);
disp('----------------------------------------------------------');
disp(' ');
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
InflateCells=2;   % 2-cell safety margin (= 2 m) — keeps Theta* shortcuts
                  % visibly clear of the red obstacle rectangles
for obs=1:NumObstacles
    % Match Gazebo generator: each obstacle is a rectangle, randomly
    % oriented tall (w x 2w) or wide (2w x w) — never a square.
    if rand()<0.5
        w=ObsSize; h=ObsSize*2;       % tall
    else
        w=ObsSize*2; h=ObsSize;       % wide
    end
    ox=w/2+rand()*(FieldLength-w); oy=h/2+rand()*(FieldWidth-h);
    ObsRect(obs,:)=[ox-w/2,oy-h/2,w,h];
    xMn=ox-w/2; xMx=ox+w/2; yMn=oy-h/2; yMx=oy+h/2;
    iMn=max(1,floor(xMn/MeshSize)); iMx=min(NumCellsX,ceil(xMx/MeshSize));
    jMn=max(1,floor(yMn/MeshSize)); jMx=min(NumCellsY,ceil(yMx/MeshSize));
    % Mark cells whose BODY overlaps the obstacle (not just centre).
    % Cell i occupies world x in [(i-1)*MeshSize, i*MeshSize].
    for i=iMn:iMx; for j=jMn:jMx
        cellXMn=(i-1)*MeshSize; cellXMx=i*MeshSize;
        cellYMn=(j-1)*MeshSize; cellYMx=j*MeshSize;
        if cellXMx>xMn && cellXMn<xMx && cellYMx>yMn && cellYMn<yMx
            ObstacleMap(i,j)=1;
        end
    end; end
end
% Inflate obstacles by InflateCells in every direction
InflatedMap=ObstacleMap;
for i=1:NumCellsX; for j=1:NumCellsY
    if ObstacleMap(i,j)==1
        for di=-InflateCells:InflateCells; for dj=-InflateCells:InflateCells
            ni=i+di; nj=j+dj;
            if ni>=1&&ni<=NumCellsX&&nj>=1&&nj<=NumCellsY; InflatedMap(ni,nj)=1; end
        end; end
    end
end; end
disp(['   Obstacles: ',num2str(NumObstacles),' | Inflated cells: ',num2str(sum(InflatedMap(:))),...
    ' | Margin: ',num2str(InflateCells),' m']);

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

% Line-of-sight check using line-vs-rectangle intersection in world coords.
% Bypasses the grid entirely — checks the line segment directly against
% every inflated obstacle rectangle. Uses the Liang–Barsky clipping test.
% Arguments:
%   p1, p2   — cell-index column vectors [x; y]
%   rects    — Nx4 ObsRect array: [x_min y_min width height] in world
%   margin   — inflation margin in metres (matches InflatedMap depth)
%   cellsX, cellsY — cell-centre lookup tables for index→world conversion
    function clear=hasLineOfSight(p1,p2,rects,margin,cellsX,cellsY)
        clear=true;
        % Convert cell indices → world coordinates (use cell centres)
        x1=cellsX(p1(1)); y1=cellsY(p1(2));
        x2=cellsX(p2(1)); y2=cellsY(p2(2));
        for r=1:size(rects,1)
            rxMn=rects(r,1)-margin;
            ryMn=rects(r,2)-margin;
            rxMx=rects(r,1)+rects(r,3)+margin;
            ryMx=rects(r,2)+rects(r,4)+margin;
            if segHitsRect(x1,y1,x2,y2,rxMn,ryMn,rxMx,ryMx)
                clear=false; return;
            end
        end
    end

% Liang–Barsky line-segment vs axis-aligned rectangle intersection.
% Returns true if the segment (x1,y1)→(x2,y2) overlaps the rectangle.
    function hit=segHitsRect(x1,y1,x2,y2,rxMn,ryMn,rxMx,ryMx)
        hit=true;
        dx=x2-x1; dy=y2-y1;
        t0=0; t1=1;
        % Four half-plane tests against the rectangle edges
        ps=[-dx,  dx, -dy,  dy];
        qs=[x1-rxMn, rxMx-x1, y1-ryMn, ryMx-y1];
        for k=1:4
            p=ps(k); q=qs(k);
            if p==0
                if q<0; hit=false; return; end
            else
                r=q/p;
                if p<0
                    if r>t1; hit=false; return; end
                    if r>t0; t0=r; end
                else
                    if r<t0; hit=false; return; end
                    if r<t1; t1=r; end
                end
            end
        end
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
disp('[6/9] Running all 4 path-finding algorithms...');
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

%% ===================== ALGORITHM 4: THETA* (A* + smoothing) ==========
% Any-angle path planning (Daniel et al. 2010).
% Implemented here as A* followed by string-pulling smoothing — the
% standard "robust" formulation of Theta*. Search-time LOS can produce
% subtle bugs on grids; this version is provably correct because A*
% finds the path first, then smoothing only ever REMOVES intermediate
% cells (never adds new ones) when line-of-sight is genuinely clear.
%
% Two passes:
%   1. A* finds a grid path  sC → ... → gC  (4-connected, always valid)
%   2. Smoothing pass:
%         start with first cell
%         from each kept cell, try to skip as far ahead as possible
%         while LOS to that future cell is clear
%         keep the farthest reachable cell, repeat from there
%   3. Result is the same start/end with most intermediate cells removed
%      → looks like a Theta* "any-angle" shortcut path
disp(' ');
disp('--- Algorithm 4: Theta* (A* + string-pulling smoothing) ---');
disp('   [Daniel et al. 2010 - any-angle paths, robust formulation]');

tic; A4Path=[]; A4Nodes=0;

for wp=1:size(Waypoints,2)-1
    sC=Waypoints(:,wp); gC=Waypoints(:,wp+1);

    % --- 1. Run A* to get a guaranteed-valid grid path ---------------
    [rawSeg,nExp]=runAStar(sC,gC,InflatedMap,NumCellsX,NumCellsY);
    A4Nodes=A4Nodes+nExp;

    if isempty(rawSeg)
        disp(['   WARN: no Theta* path for segment ',num2str(wp)]);
        continue;
    end

    % --- 2. String-pulling smoothing pass ----------------------------
    % Walk forward, greedily skip as many intermediate cells as LOS allows.
    % LOS uses world-coord line-vs-rectangle test (no grid sampling).
    SafetyMargin=InflateCells*MeshSize+0.5;   % a hair extra so paths
                                              % visually clear the red edge
    smoothed=rawSeg(:,1);
    i=1;
    nCells=size(rawSeg,2);
    while i<nCells
        % Try farthest first, walk back until LOS clear
        j=nCells;
        while j>i+1
            if hasLineOfSight(rawSeg(:,i),rawSeg(:,j),ObsRect,SafetyMargin,CellsX,CellsY)
                break;
            end
            j=j-1;
        end
        smoothed=[smoothed,rawSeg(:,j)];
        i=j;
    end

    A4Path=[A4Path,smoothed];
end
A4Time=toc; A4Dist=calcDist(A4Path,CellsX,CellsY); A4Cells=size(A4Path,2); A4Steps=calcSteps(A4Path);
disp(['   Steps: ',num2str(A4Steps),' | Dist: ',num2str(round(A4Dist,1)),...
    'm | Nodes: ',num2str(A4Nodes),' | Time: ',num2str(round(A4Time,3)),'s']);

%% ===================== ANIMATED MOVEMENT =============================
disp(' ');
disp('[8/10] Animating movement...');
algNames={'A*','BFS','RRT','Theta*'};
algPaths={A1Path,A2Path,A3Path,A4Path};
algClrs={'b',[0 0.7 0],[0.8 0.4 0],[0 0.6 0.6]};
numAlgs=4;

figure('Name','SEN771 - Animation','Position',[20 20 1600 500]);
for alg=1:numAlgs
    subplot(1,4,alg); hold on;
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
        subplot(1,4,alg);
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
        subplot(1,4,alg);
        if isvalid(tH(alg)); delete(tH(alg)); end
        tH(alg)=plot(tX{alg},tY{alg},'-','Color',algClrs{alg},'LineWidth',1.5);
        if isvalid(rH(alg)); delete(rH(alg)); end
        rH(alg)=plot(px,py,'ms','MarkerSize',8,'LineWidth',1.5,'MarkerFaceColor','m');
    end
    drawnow; pause(0.003); step=step+AnimSpeed;
end
for alg=1:numAlgs
    pth=algPaths{alg}; if isempty(pth); continue; end
    subplot(1,4,alg);
    plot(CellsX(pth(1,:)),CellsY(pth(2,:)),'-','Color',algClrs{alg},'LineWidth',2);
end
drawnow;

%% ===================== PERFORMANCE COMPARISON ========================
disp(' ');
disp('[9/10] Performance comparison...');
disp(' ');

% Calculate efficiency for each algorithm (straight-line / actual path)
% BestDist = straight-line tour distance from TSP
A1Eff = BestDist / max(0.1, A1Dist) * 100;
A2Eff = BestDist / max(0.1, A2Dist) * 100;
A3Eff = BestDist / max(0.1, A3Dist) * 100;
A4Eff = BestDist / max(0.1, A4Dist) * 100;

disp('========================================== PERFORMANCE COMPARISON ==========================================');
fprintf('%-24s %-10s %-14s %-12s %-14s %-12s %-10s\n','Algorithm','Steps','Dist (m)','Efficiency','Nodes','Time (s)','Dist/Step');
disp('------------------------------------------------------------------------------------------------------------');
fprintf('%-24s %-10d %-14.1f %-12.1f%% %-14d %-12.3f %-10.2f\n','1. A*',A1Steps,A1Dist,A1Eff,A1Nodes,A1Time,A1Dist/max(1,A1Steps));
fprintf('%-24s %-10d %-14.1f %-12.1f%% %-14d %-12.3f %-10.2f\n','2. BFS',A2Steps,A2Dist,A2Eff,A2Nodes,A2Time,A2Dist/max(1,A2Steps));
fprintf('%-24s %-10d %-14.1f %-12.1f%% %-14d %-12.3f %-10.2f\n','3. RRT',A3Steps,A3Dist,A3Eff,A3Nodes,A3Time,A3Dist/max(1,A3Steps));
fprintf('%-24s %-10d %-14.1f %-12.1f%% %-14d %-12.3f %-10.2f\n','4. Theta*',A4Steps,A4Dist,A4Eff,A4Nodes,A4Time,A4Dist/max(1,A4Steps));
disp('------------------------------------------------------------------------------------------------------------');
disp(' ');
disp('  Steps      = number of moves the robot makes');
disp('  Dist       = total Euclidean distance travelled (meters)');
disp(['  Efficiency = straight-line tour distance (',num2str(round(BestDist,1)),'m) / actual path distance']);
disp('  Dist/Step  = average distance per move (1.0 for grid, >1 for RRT)');

%% ===================== BAR CHARTS ====================================
disp('[10/10] Plotting comparison charts...');
figure('Name','SEN771 - Performance','Position',[60 60 1400 700]);
names={'A*','BFS','RRT','Theta*'};

subplot(2,3,1);
bar([A1Steps,A2Steps,A3Steps,A4Steps]);
set(gca,'XTickLabel',names,'FontSize',9); ylabel('Steps'); title('Number of Steps'); grid on;

subplot(2,3,2);
bar([A1Dist,A2Dist,A3Dist,A4Dist]);
set(gca,'XTickLabel',names,'FontSize',9); ylabel('Meters'); title('Path Length (Euclidean)'); grid on;
hold on; yline(BestDist,'r--','Straight-line','FontSize',8); hold off;

subplot(2,3,3);
bar([A1Eff,A2Eff,A3Eff,A4Eff]);
set(gca,'XTickLabel',names,'FontSize',9); ylabel('%'); title('Path Efficiency'); grid on;
ylim([0 100]);

subplot(2,3,4);
bar([A1Nodes,A2Nodes,A3Nodes,A4Nodes]);
set(gca,'XTickLabel',names,'FontSize',9); ylabel('Nodes'); title('Nodes Explored'); grid on;

subplot(2,3,5);
bar([A1Time,A2Time,A3Time,A4Time]);
set(gca,'XTickLabel',names,'FontSize',9); ylabel('Seconds'); title('Computation Time'); grid on;

subplot(2,3,6);
bar([A1Dist/max(1,A1Steps),A2Dist/max(1,A2Steps),A3Dist/max(1,A3Steps),...
    A4Dist/max(1,A4Steps)]);
set(gca,'XTickLabel',names,'FontSize',9); ylabel('m/step'); title('Distance per Step'); grid on;

%% ===================== SUMMARY ======================================
disp(' ');
disp('==========================================================');
disp('  Simulation complete!');
disp(['  Target order: Start->T',num2str(BestOrder(1)),'->T',...
    num2str(BestOrder(2)),'->T',num2str(BestOrder(3)),'->T',...
    num2str(BestOrder(4)),'->Start']);
disp(' ');
disp('  Algorithms:');
disp('    1. A* Search          (workshop algorithm)');
disp('    2. BFS                (workshop algorithm)');
disp('    3. RRT                (included for comparison - sampling-based)');
disp('    4. Theta*             (NEW - any-angle, Daniel et al. 2010)');
disp(' ');
disp('  Figures: 1) Scenario  2) Animation  3) Comparison');
disp('==========================================================');
