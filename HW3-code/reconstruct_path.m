function [ path, cost ] = reconstruct_path(cameFrom, goal)
    current = transpose(goal);
    totalPath = [];
    cost = 0;
    prev = [ current(2), current(1) ];
    ignore = true;
    while ~isnan(current) 
        % y then x for lookup
        if ~ignore % ignore first round
            cost = cost + distance(transpose(current), prev);
        end
        totalPath = [ totalPath, current ];
        prev = [ current(1), current(2) ];
        current = squeeze(cameFrom(current(2), current(1), :));
        ignore = false;
        
    end
    path = transpose(totalPath);
    