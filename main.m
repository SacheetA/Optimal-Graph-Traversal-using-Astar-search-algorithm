clc, clear all; close all

%% adjacecny matrix computation
% Write your code compute the symmetric adjacecny matrix adjMat
P = [5 inf inf inf inf inf inf 3 10 inf inf inf inf 4 inf inf 7 inf inf inf 2 1 inf 8 inf inf  2 inf inf inf 5 inf inf 1 inf inf];
adjMat = squareform(P);
for i = 1:length(adjMat)
    for j = 1:length(adjMat)
        if adjMat(i,j) == inf
            adjMat(i,j) = 0;
        end
    end
end

%% creating nodes for the A* algorithm
numNodes = 9; % number of nodes;
nodesList = createAstarNodes(numNodes);

%% creating the heuristic cost array
% Note that the heuristic cost is specific for a particular goal node. 

goal = 6; %specify the goal node;

% fill in the heuristic cost to go from every node to the goal node.
% H is a 1xnumNodes array of heuristic costs
H = [4 3 2 2 1 0 1 2 3];  %for 6

%H = [1 0 1 3 2 3 2 1 2];   %for 2

% Ex. if goal = 6, then H = [4 3 2 2 1 0 1 2 3] if we consider the least number
% of hops as the heuristic cost.

%% implementing A* algorithm
start = 1; %specify the starting node;

%obtain optimal path using A*
[path] = Astar(nodesList,start,goal,adjMat,H);

%% compute path length
path_length = 0;
for i = 1:(length(path)-1)
    path_length = path_length + adjMat(path(i), path(i+1));
end

%% shortest path and its length from the given graph
path
path_length

