function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an mx3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path. The first
%   row is start and the last row is goal. If no path is found, PATH is a
%   0x3 matrix. Consecutive points in PATH should not be farther apart than
%   neighboring voxels in the map (e.g. if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of nodes that were expanded while performing the search.
%
% paramaters:
%   map     - the map object to plan in
%   start   - 1x3 vector of the starting coordinates [x,y,z]
%   goal:   - 1x3 vector of the goal coordinates [x,y,z]
%   astar   - boolean use astar or dijkstra

if nargin < 4
    astar = false;
end


startLinIndex=pos2ind(map,start);
goalLinIndex=pos2ind(map,goal);
gSubs=pos2sub(map,goal);
[nI, nJ, nK]= size(map.occgrid);
costF=Inf(nI, nJ, nK);
parent=NaN(numel(map.occgrid),1);
costF(startLinIndex)=0;
[iInd, jInd, kInd] = ndgrid(1:nI,1:nJ,1:nK);
heuristic=zeros(size(costF));
if astar
    heuristic=sqrt((map.res_xyz(1)*(iInd-gSubs(1))).^2+(map.res_xyz(2)*(jInd-gSubs(2))).^2+(map.res_xyz(3)*(kInd-gSubs(3))).^2);
end
parent(startLinIndex)=startLinIndex;
isGoalInQ=true;
num_expanded = 0;
QlinInd=(1:numel(map.occgrid))';
QisNotExploredMult=ones(size(QlinInd));
Qpriortzd=startLinIndex;

adjFactor=[ 1 0 0;          0 1 0;         -1 0 0;            0 -1 0;           1 1 0;             -1 -1 0;            -1 1 0;       1 -1 0; ...
    1 0 1;          0 1 1;          -1 0 1;           0 -1 1;           1 1 1;            -1 -1 1;           -1 1 1;           1 -1 1;           0 0 1; ...
    1 0 -1;          0 1 -1;        -1 0 -1;          0 -1 -1;          1 1 -1;           -1 -1 -1;          -1 1 -1;          1 -1 -1;          0 0 -1];

euclDist=[ map.res_xyz(1); map.res_xyz(2); map.res_xyz(1); map.res_xyz(2);sqrt(map.res_xyz(1)^2+map.res_xyz(2)^2)  ; sqrt(map.res_xyz(1)^2+map.res_xyz(2)^2) ;sqrt(map.res_xyz(1)^2+map.res_xyz(2)^2) ;sqrt(map.res_xyz(1)^2+map.res_xyz(2)^2); ...
    sqrt(map.res_xyz(1)^2+map.res_xyz(3)^2) ;sqrt(map.res_xyz(2)^2+map.res_xyz(3)^2) ;sqrt(map.res_xyz(1)^2+map.res_xyz(3)^2) ;sqrt(map.res_xyz(2)^2+map.res_xyz(3)^2) ;sqrt(map.res_xyz(1)^2+map.res_xyz(2)^2+map.res_xyz(3)^2) ;sqrt(map.res_xyz(1)^2+map.res_xyz(2)^2+map.res_xyz(3)^2) ;sqrt(map.res_xyz(1)^2+map.res_xyz(2)^2+map.res_xyz(3)^2) ; sqrt(map.res_xyz(1)^2+map.res_xyz(2)^2+map.res_xyz(3)^2) ; map.res_xyz(3);...
    sqrt(map.res_xyz(1)^2+map.res_xyz(3)^2)  ; sqrt(map.res_xyz(2)^2+map.res_xyz(3)^2) ;sqrt(map.res_xyz(1)^2+map.res_xyz(3)^2) ;sqrt(map.res_xyz(2)^2+map.res_xyz(3)^2);sqrt(map.res_xyz(1)^2+map.res_xyz(2)^2+map.res_xyz(3)^2) ; sqrt(map.res_xyz(1)^2+map.res_xyz(2)^2+map.res_xyz(3)^2) ;sqrt(map.res_xyz(1)^2+map.res_xyz(2)^2+map.res_xyz(3)^2) ;sqrt(map.res_xyz(1)^2+map.res_xyz(2)^2+map.res_xyz(3)^2); map.res_xyz(3)];
% minCost=0;

alwaysInfCostInd=find(map.occgrid==1,1);
if isempty(alwaysInfCostInd)
    costF(:,:,:,2)=Inf;
    parent(:,:,:,2)=0;
    alwaysInfCostInd=sub2ind(size(costF),1,1,1,2);
end

if map.occgrid(startLinIndex)==1 || map.occgrid(goalLinIndex)==1
    path=[];
else
    while isGoalInQ && ~isempty(Qpriortzd)
        [~,minInd]=min(costF(Qpriortzd));
        ind2explore=Qpriortzd(minInd);
        Qpriortzd(minInd)=[];
%         [minCost,minInd]=min(costF(QlinInd).*QisNotExploredMult);
        QisNotExploredMult(ind2explore)=NaN;
        isGoalInQ=QisNotExploredMult(goalLinIndex)==1;
        [iExpl, jExpl, kExpl]=ind2sub(size(map.occgrid),ind2explore);
        neighbIJK=[iExpl, jExpl, kExpl]+adjFactor;
        neighbMult=ones(length(neighbIJK),1);
        neighbMult(neighbIJK(:,1)<=0 | neighbIJK(:,2)<=0 | neighbIJK(:,3)<=0 | neighbIJK(:,1)>nI | neighbIJK(:,2)>nJ | neighbIJK(:,3)>nK,:)=NaN; %remove out of bounds
        neighbLinInd=sub2ind(size(map.occgrid),neighbIJK(:,1).*neighbMult,neighbIJK(:,2).*neighbMult,neighbIJK(:,3).*neighbMult);
        tmp=neighbLinInd;
        tmp(isnan(tmp))=startLinIndex;
        neighbMult(logical(map.occgrid(tmp)))=NaN; %remove obstacles by setting to 0, occgrid- 0: free, 1:obstacle
        tmp(tmp==startLinIndex)=goalLinIndex;
        heuristicLocal=heuristic(tmp);
        neighbLinInd=neighbLinInd.*neighbMult;
        num_expanded=num_expanded+1;
        costInit=costF(ind2explore);
        newCosts=euclDist.*neighbMult+costInit+heuristicLocal;
%         neighbLinInd(isnan(neighbLinInd))=find(isinf(costF),1); % make all the neighbors that are out of bounds have inf cost
        neighbLinInd(isnan(neighbLinInd))=alwaysInfCostInd;
        [newCosts,newParentsIndAdd]=min([costF(neighbLinInd),newCosts],[],2);
        Qpriortzd=[Qpriortzd;neighbLinInd(isinf(costF(neighbLinInd)) & ~isinf(newCosts))];
%         Qpriortzd=[Qpriortzd;neighbLinInd(newParentsIndAdd==2)];
        n=length(newParentsIndAdd);
        newParentsIndAdd(newParentsIndAdd==2)=n;
        newParentsIndAdd(newParentsIndAdd==1)=0;
        parentOpts=zeros(n*2,1);
        parentOpts(1:n)=parent(neighbLinInd);
        parentOpts(n+1:end)=ones(size(neighbLinInd))*ind2explore;
        newParents=parentOpts((1:n)'+newParentsIndAdd);        
        costF(neighbLinInd)=newCosts;
        parent(neighbLinInd)=newParents;
    end
    
    path=zeros(0,3);
    if costF(goalLinIndex)<Inf
        parentInd=goalLinIndex;
        path=goal;
        while parentInd~=startLinIndex
            path=[path;ind2pos(map,parentInd)];
            parentInd=parent(parentInd);
        end
        path=[path;ind2pos(map,parentInd)];
        path=[path;start];
        path=[path(end:-1:1,1),path(end:-1:1,2),path(end:-1:1,3)];
    end
    
end


end
