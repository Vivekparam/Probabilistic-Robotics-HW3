function [minX, minY] = min_state(f_score_map)
% returns minX minY of the state on the map which has the lowest f_score
minX = 0
minY = 0
minCost = Inf
[width, height] = size(f_score_map) % get width and height

for col=1:width
  for row=1:height
    if f_score_map(row, col) < minCost
        minCost = f_score_map(minY, minX)
        minX = row
        minY = col
    end
  end
end
          
 % done?