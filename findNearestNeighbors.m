function N = findNearestNeighbors(i,adjMat)
% returns the nearest neighbours of node i in the graph described by adjMat
p = 1;
for l=1:length(adjMat)
    if (adjMat(i,l) ~= inf && adjMat(i,l) ~= 0)
        N(p) = l;           %equated to index and not number '1'
        p = p+1; 
    end
end
% returns the nearest neighbours of node i in the graph described by adjMat
end