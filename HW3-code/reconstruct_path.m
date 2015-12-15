function path = reconstruct_path(cameFrom, current)
    totalPath = [ current ]
    while ~isnan(current) 
        % y then x for lookup
        current = cameFrom(current(2), current(1))4
        totalPath = [ current, totalPath ]