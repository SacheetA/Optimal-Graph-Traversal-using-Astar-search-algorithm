function [path] = Astar(nList,start,goal,adjMat,H)
% Create vectors for the f,g and h costs and initialize them
% The fcosts are initialized with a high value initially in order to avoid
% unexplored nodes from being selected for expansion. For example, if you
% initialize the fCosts for all nodes with zero, then after the first
% iteration, the node with zero cost will be selected. This node may not
% even be in the list of nearest neighbors!

%% ALSO CHECK THE PSEUDO CODE FOR A* algorithm at the end of the file. 

fCost = Inf*ones(size(nList,2),1);
gCost = Inf*ones(size(nList,2),1);
hCost = zeros(size(nList,2),1);

% unvisited contains the nodes which are alive.
% The unvisited contains the start node ID to begin with.
unvisited = [start];

% visited contains list of the dead nodes.
%The visited list is empty initially.
visited = [];

% Begin the algorithm with the start node
curNode = nList(start); % current or active node

%Initialize the gCost, hCost and fCost for the starting node
%gCost: Is the cost to reach a given node from the node you started from
%hCost: Is the heuristic cost to reach the goal
%fCost: Is the sum of gCost and hCost

gCost(start,1) = 0;
hCost(start,1) = H(start);
fCost(start,1) = gCost(start,1) + hCost(start,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%******************* This is the main Astar loop**************************
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%It is necessary to keep running the loop until the goal ID
%is found in the visited

while(~ismember(goal,visited))

    curId = curNode.ID;   %Extract the node ID of the current node from the nodeStructure

    % Remove the current node ID from the open list and put it in the closed list\
    unvisited = setdiff(unvisited,curId);

    %Add the node you are evaluating to the closed list
    visited = [visited, curId];
    
    %Check if the current node is the goal node. 
    if (curId == goal) 
      continue 
    end
    
    %The current node is not the goal node. So, check the next door
    %neighbors to proceed towards the goal.
    curNode.nextDoorNeighbors = findNearestNeighbors(curId,adjMat);
    
    for nIdx = 1:1:numel(curNode.nextDoorNeighbors)
        nDn = curNode.nextDoorNeighbors(nIdx);
        if(ismember(nDn,visited))
           %No action required
        else
            temp_gCost = gCost(curId) + adjMat(curId,nDn); 
            if(~ismember(nDn,unvisited))
                unvisited = [unvisited, nDn];
                hCost(nDn) = H(nDn);
                updateGCost = true;
            elseif temp_gCost < gCost(nDn)
                updateGCost = true;
            else
                updateGCost = false;
            end
            if(updateGCost)
                gCost(nDn) = temp_gCost;
                fCost(nDn) = hCost(nDn) + gCost(nDn);
                nList(nDn).parent =  curId;
            end
        end
    end
    %Choose the node in the open list with the least F_Cost for the next iteration
    fC = fCost(unvisited);
    [~,minId] = min(fC);
    
    minFcostId = minId;
    curNode = unvisited(minFcostId); 
    curNode = nList(curNode);
    
end

%After the goal has been found, the path needs to be reconstructed by
%backtracking the parent nodeIDs until the startnode ID is found
nParent = nList(goal).parent;
nodeIDPath = [goal];
while(nParent ~= start)
    nodeIDPath = [nParent nodeIDPath];
    nParent = nList(nParent).parent;
end
% This array contains a list of nodeID path.
path = [start nodeIDPath];


%%*************************************************************************
%     PSEUDO CODE FOR A* algorithm from WIKIPEDIA.ORG
%%*************************************************************************

% function A*(start,goal)
%     closedset := the empty set    // The set of nodes already evaluated.
%     openset := {start}    // The set of tentative nodes to be evaluated, initially containing the start node
%     came_from := the empty map    // The map of navigated nodes.
%  
%     g_score := map with default value of Infinity
%     g_score[start] := 0    // Cost from start along best known path.
%     // Estimated total cost from start to goal through y.
%     f_score = map with default value of Infinity
%     f_score[start] := g_score[start] + heuristic_cost_estimate(start, goal)
%      
%     while openset is not empty
%         current := the node in openset having the lowest f_score[] value
%         if current = goal
%             return reconstruct_path(came_from, goal)
%          
%         remove current from openset
%         add current to closedset
%         for each neighbor in neighbor_nodes(current)
%             if neighbor in closedset
%                 continue
%  
%             tentative_g_score := g_score[current] + dist_between(current,neighbor)
% 
%             if neighbor not in openset or tentative_g_score < g_score[neighbor] 
%                 came_from[neighbor] := current
%                 g_score[neighbor] := tentative_g_score
%                 f_score[neighbor] := g_score[neighbor] + heuristic_cost_estimate(neighbor, goal)
%                 if neighbor not in openset
%                     add neighbor to openset
%  
%     return failure
% 
% function reconstruct_path(came_from,current)
%     total_path := [current]
%     while current in came_from:
%         current := came_from[current]
%         total_path.append(current)
%     return total_path

    
    
    
        
    
    
