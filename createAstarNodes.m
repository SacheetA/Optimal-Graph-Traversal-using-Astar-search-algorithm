function [nodes] = createAstarNodes(numNodes)

    for idx=1:1:numNodes
        nodes(idx).ID = idx;
        nodes(idx).nextDoorNeighbors = [];
        nodes(idx).parent = [];
    end

end