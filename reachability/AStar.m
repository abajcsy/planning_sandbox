classdef AStar
    %ASTAR Planner
    
    properties
        lowEnv      % lower (x,y) corner of environment
        upEnv       % upper (x,y) corner of environment
        simWidth    % width of astar grid
        simHeight   % height of astar grid
        resX        % resolution in X (realWidth/simWidth)
        resY        % resolution in Y (realHeight/simHeight)
    end
    
    methods
        %% Constructor.
        function obj = AStar(lowEnv, upEnv, simWidth, simHeight)
            obj.lowEnv = lowEnv;
            obj.upEnv = upEnv;
            obj.simWidth = simWidth;
            obj.simHeight = simHeight;
            
            obj.resX = (obj.upEnv(1) - obj.lowEnv(1))/obj.simWidth;
            obj.resY = (obj.upEnv(2) - obj.lowEnv(2))/obj.simHeight;
        end
        
        %% Plans collision-free waypoints from xcurr to goalXY
        % A* (star) Pathfinding
        % Inputs:
        %   start [vector]  - (X,Y) current grid location of car
        %   goal [vector]   - (X,Y) grid location of goal
        % Output:
        %   waypts [cell array] - (X,Y) waypts along shortest path from start to goal
        function waypts = plan(start, goal)
            % Initialize both open and closed list to empty list of Nodes.
            openList = {};
            closedList = {};
            
            % Create start & end node (with default zero for total cost, etc.)
            startNode = Node(NaN, start);
            goalNode = Node(NaN, goal);
            
            % Add the start node to the openList (leave it's totalCost at zero)
            openList{end+1} = startNode;
            
            % Loop until you find the end
            while ~isempty(openList)
                % Let the currentNode equal the node with the least totalCost
                currNode = openList{1};
                currIdx = 0;
                for k=1:length(openList)
                    node = openList{k};
                    if node.totalCost < currNode.totalCost
                        currNode = node;
                        currIdx = k;
                    end
                end
                
                % remove the currentNode from the openList
                openList(currIdx) = [];
                % add the currentNode to the closedList
                closedList{end+1} = currNode;
                
                % if currentNode is the goal
                if currNode.equal(goalNode)
                    % Congratz! You've found the end! Backtrack to get path
                    path = {};
                    curr = currNode;
                    while ~isnan(current)
                        path{end+1} = curr.position;
                        curr = curr.parent;
                    end
                    % Return reversed path
                    waypts = fliplr(path);
                end
                    
                % // Generate children
                % let the children of the currentNode equal the adjacent nodes
                children = obj.getChildren(currNode);
                
                for k=1:length(children)
                % for each child in the children
                    child = children{k};
                    
                    % check if child is on the closedList
                    for j=1:length(closedList)
                        closedNode = closedList{j};
                        if child.equal(closedNode)
                            % continue to beginning of for loop
                            continue;
                        end
                    end
                        
                    % // Create the dist to start, goal, and total cost values
                    child.dToStart = currNode.dToStart + currNode.distance(child);
                    child.dToEnd = currNode.distance(goalNode);
                    child.totalCost = child.dToStart + child.dToGoal;
                    
                    % // Child is already in openList
                    for j=1:length(openList)
                        openNode = openList{j};
                        % if child.position is in the openList's nodes
                        % positions and if the child.dToStart is higher than the 
                        % openList node's dToStart
                        if child.equal(openNode) && child.dToStart > openNode.dToStart
                            % continue to beginning of for loop
                            continue; 
                        end
                    end

                    % Add the child to the openList
                    openList{end+1} = child;
                end
            end
        end
        
        %% Returns cell array of adjacent (children) Nodes.
        function children = getChildren(obj, currNode)
            
            children = {};
            adjacentSquares = {[0, -1], [0, 1], [-1, 0], [1, 0], [-1, -1], [-1, 1], [1, -1], [1, 1]};
            for k=1:length(adjacentSquares) 
                newPos = adjacentSquares{k};
                
                % Get node position
                nodePos = [currNode.position(1) + newPos(1), currNode.position(2) + newPos(2)];

                % Make sure within range of world
                if nodePos(1) > obj.simWidth || nodePos(1) < 1 || ... 
                    nodePos(2) > obj.simHeight || nodePos(2) < 1
                    continue
                end

                %Make sure walkable terrain
                if obj.inCollision(nodePos)
                    continue
                end
                
                %Create new node
                newNode = Node(currNode, nodePos);

                %Append
                children{end+1} = newNode;
            end
        end
        
        %% Checks if XY position is collision-free.
        function bool = inCollision(obj, position)
            % TODO -- THIS JUST ALWAYS RETURNS FALSE!
            bool = false;
        end
        
        %% Converts (x,y,theta) to XY sim coordinates.
        % Inputs:
        %   xcurr [vector] - (x,y,theta) current location of car
        % Output:
        %   XY [vecotr] - (x,y) coords in gridworld
        function XY = realToSim(xcurr)
            XY = [floor(xcurr(1)/self.resX)+1; floor(xcurr(2)/self.resY)+1];
        end

    end
end

