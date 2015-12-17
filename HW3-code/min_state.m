function [minX, minY] = min_state(f_score_map, displaymap)
% returns minX minY of the open state on the map which has the lowest f_score
minX = 1;
minY = 1;
minCost = Inf;
[height, width] = size(f_score_map); % get width and height

for col=1:width
  for row=1:height
    if f_score_map(row, col) < minCost
        if displaymap(row, col) == 2
            minCost = f_score_map(row, col);
            minX = col;
            minY = row;
        end
    end
  end
end


          
 % done?