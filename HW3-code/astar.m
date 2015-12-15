function[fpath, cost, displaymap] = astar(envmap, start, goal, epsilon)
% Implements the A-star motion planner for a point-robot to find a
% collision-free path from start to goal
% INPUTS:
% envmap  - Map of the environment, envmap(y,x) = 1 means the cell
%           (x,y) is an obstacle, envmap(y,x) = 0 means it is free
% start   - Robot start [x,y] = [col,row]
% goal    - Robot goal [x,y] = [col,row]
% epsilon - (Optional) Epsilon for epsilon a-star >= 1.0 (Default = 1.0)
% OUTPUTS:
% fpath      - Final collision-free path (N x 2, each row is [x,y])
% cost       - Cost of the final path
% displaymap - Map highlighted with closed & open states from a-star

if nargin < 3
    error('Need to pass in map, start and goal');
end

if nargin < 4
    epsilon = 1.0;
end
if epsilon < 0
    error('Epsilon has to be >= 0.0');
end
fprintf('Using epsilon = %f \n', epsilon);

% Have a map for display
displaymap = envmap;
fg = figure(101); hold on;
imgg = imagesc(displaymap);
t1 = text(start(1), start(2), 'S'); set(t1,'Color','r','Fontsize',15);
t2 = text(goal(1), goal(2), 'G'); set(t2,'Color','g','Fontsize',15);
title('A-star planning');
xlim([1,size(displaymap,2)]);
ylim([1,size(displaymap,1)]);

%% READ -> STUDENT TODO: 
% Implement A-star to find a collision free path. 
%  a) Use cost of g+epsilon*h, epsilon > 1 for epsilon a-star
%  where the heuristic is Euclidean distance to the goal. Each transition
%  has a cost of 1.0 irrespective of the action taken (diagonal actions
%  have same cost). Use an 8-connected neighbourhood for A-star.
%  b) A state is an obstacle if:
%       envmap(state(2), state(1)) = 1
%     A state is free if:
%       envmap(state(2), state(1)) = 0
%  c) To check if a state is within limits, use the function "checkLimits"
%  d) Display the closed/open states for a-star as the search progresses.
%  You can easily do this by setting these states to have the correct value
%  in the variable "displaymap". For example, open states can have
%  value = 2 & closed states can have value = 3. Note that value of 0 means
%  free & 1 means obstacles.
%     displaymap(openState(2), openState(1)) = 2
%     displaymap(closedState(2), closedState(1)) = 3
%  
% (x,y) = (col,row) in MATLAB sense

% use euclidean distance from goal as heuristic

[width, height] = size(envmap) % get width and height

curState = start
% openStates = []
cameFrom = zeroes(width, height, 2) % width, height, and each has a (x, y) camefrom

g_score = Inf(width, height)
g_score(start) = 0

f_score = Inf(width, height)
f_score(start) = distance(start, goal)


ct = 0;
while(curState(1) ~= goal(1) && curState(2) ~= goal(2))
    # extract x and y
    curX = curState(1)
    curY = curState(2)
    
    % Mark current state as closed
    displaymap(curState(2), curState(1)) = 3
    
   
    for col=(curX - 1):(curX + 1)
      for row=(curY - 1):(curY + 1)
      
        % if outside bounds, ignore it
        if ~checkLimits(envmap, [col, row])
            continue
        end
        
        % if its an obstacle, ignore it  
        if envmap(row, col) == 1
            continue
        end    
        % if closed already, ignore it
        if displaymap(row, col) == 3
            continue
        end   
        % calculate length of this path
        tentative_g_score = g_score(curY, curX) + 1 % cost of motion always 1
        
        if displaymap(row, col) ~= 2
            displaymap(row, col) = 2 % if not already open, open this node
        else if tentative_g_score >= g_score(row, col) % this is a worse path to this location, move on
            continue
        end 
        % if we get here, then this is our best path so far
        cameFrom(row, col) = [curX, curY]
        g_score(row, col) = tentative_g_score
        f_score(row, col) = tentative_g_score + distance([col, row], goal)
            
        curState = min_state(f_score)
      end
    end 
    % TODO: Run till goal state is reached or there are no open states
    % If state is open -> displaymap(state(2), state(1)) = 2
    % If state is closed -> displaymap(state(2), state(1)) = 3
    
    % Display intermittently - assumes that displaymap has been modified
    % properly by the student
    if ~mod(ct,200)
        figure(fg);
        set(imgg, 'CData', displaymap);
        drawnow;
    end
    
    % Increment counter
    ct = ct+1;
end

% Final display before exit
figure(fg);
set(imgg, 'CData', displaymap);
drawnow;
fprintf('Number of closed states: %d \n', sum(sum(displaymap == 3)));

% Display path
figure(fg);
plot(fpath(:,1), fpath(:,2), 'k.-', 'Linewidth',2);
end