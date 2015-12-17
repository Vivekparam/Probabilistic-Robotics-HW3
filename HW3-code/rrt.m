function[fpath, cost] = rrt(envmap, start, goal, deltaStep)
% Implements the RRT motion planner for a point-robot to find a
% collision-free path from start to goal
% INPUTS:
% envmap    - Map of the environment, envmap(y,x) = 1 means the cell
%           (x,y) is an obstacle, envmap(y,x) = 0 means it is free
% start     - Robot start [x,y] = [col,row]
% goal      - Robot goal [x,y] = [col,row]
% deltaStep - (Optional) Approx. number of cells by which to extend the graph in
%             direction of the nearest-neigbour on the graph (Default: 10)
% OUTPUTS:
% fpath     - Final collision-free path (N x 2, each row is [x,y])
% cost      - Cost of the final path (sum of squared distances between nodes on the path)

if nargin < 3
    error('Need to pass in map, start and goal');
end

if nargin < 4
    deltaStep = 10;
end
fprintf('Deltastep = %f \n',deltaStep);

% Figure
fg = figure(101); hold on;
imagesc(envmap); 
t1 = text(start(1), start(2), 'S'); set(t1,'Color','r','Fontsize',15);
t2 = text(goal(1), goal(2), 'G'); set(t2,'Color','g','Fontsize',15);
title('RRT planning');
xlim([1,size(envmap,2)]);
ylim([1,size(envmap,1)]);

%% READ -> STUDENT TODO:
% Implement RRT to find a collision free path. Note that the edges also
% have to be collision-free, not just the nodes.
%   a) Sample states at random 99% of time, 1% sample goal state
%   b) Extend the graph by "deltaStep" steps from the nearest
%   neighbour of the sample in the direction of the sample (along the
%   straight line). If sample is within deltaStep distance, use it directly
%   c) Approx. Collision checking for the (approx.) straight line path
%   between two states can be done using the function "collcheckstline".
%   This also returns the states along the straight line path.
%   d) A state is an obstacle if:
%       envmap(state(2), state(1)) = 1
%      A state is free if:
%       envmap(state(2), state(1)) = 0
%   e) Run till you find a state on the graph rounding which you get the
%   goal state
%   f) Display the progression of the graph generation for RRT (figure fg)
% 
% (x,y) = (col,row) in MATLAB sense
% Use checkLimits.m to check for limits for the point robot. Round-off
% states for collision checking.
ct = 0;
goalStateNotReached = true;
[width, height] = size(envmap);


cameFrom = NaN(width, height, 2); % width, height, and each has a (x, y) camefrom



rrt = {}; % tree
node.p = start;
node.parent = node; % it is its own parent
rrt{end+1} = node;

fprintf('begin!');
while(goalStateNotReached)
    
    % TODO: Run planning till goal reached
    
    % generate a random state 99% of the time, sample 
    % the goal the 1% of the time
    if mod(ct, 100) == 0
        randState = goal;
    else
        randState = rand(2,1);
        randState(1) = (randState(1) * width);
        randState(2) = (randState(2) * height);
    end
    
    if incollission_node(randState, envmap)
        continue % this node is inside of an obstacle
    end
    
    % find nearest neighbor to state
    nearState = nearest_neighbor(randState, rrt);
    
    dist = distance(nearState, randState);
    
    if dist < deltaStep
        newState = [ round(randState(1)), round(randState(2)) ] ;
    else
        distRatio = deltaStep / dist;
        xDist = (randState(1) - nearState(1)) * distRatio;
        yDist = (randState(2) - nearState(2)) * distRatio;
        newX = nearState(1) + xDist;
        newY = nearState(2) + yDist;
        newState = [ round(newX), round(newY) ]; % draw edge to here
    end
    
    if incollission_node(newState, envmap)
        continue
    end
    
    plot([ nearState(1) newState(1) ], [ nearState(2), newState(2) ], 'Color', [1, 0, 0]);
    % drawnow;
    % make sure edge wont collide
    
    
    % add node to tree
    newNode.p = newState;
    newNode.parent = nearState;
    rrt{end+1} = newNode;
    
    % keep track of where we came from
    cameFrom(newState(2), newState(1), :) = [nearState(1) , nearState(2)];
    
    if newState(1) == goal(1) && newState(2) == goal(2)
        % we did it!
        goalStateNotReached = false;
    end
    
    % Display intermittently - assumes that student plots the graph
    if ~mod(ct,200)
        figure(fg);
        drawnow;
    end
    
    % Increment counter
    ct = ct+1; 
end

[fpath, cost] = reconstruct_path(cameFrom, goal);
fprintf('done!');
% Draw a final time before exiting
figure(fg);
drawnow;

end