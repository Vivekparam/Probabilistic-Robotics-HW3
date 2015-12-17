function nn = nearest_neighbor(state, rrt)
    % returns the nearest node in rrt to the state
    mindist = Inf;
    for i=1:length(rrt)
        dist = distance(rrt{i}.p, state);
        if (i==1) || (dist < mindist)
            mindist = dist;
            minnode = rrt{i}.p;
        end
    end
    nn = minnode;