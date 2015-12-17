function result = incollission_node(node,envmap)

global checkcount;
checkcount = checkcount + 1;

[height, width] = size(envmap); % get width and height

for col=1:width
  for row=1:height    % calculate distance between ith robot ball center and jth obstacle
    % ball center
    %dist = sqrt((rob.ball{i}.p(1)-obst.ball{j}.p(1))^2+(rob.ball{i}.p(2)-obst.ball{j}.p(2))^2+(rob.ball{i}.p(3)-obst.ball{j}.p(3))^2);
    if envmap(row, col) ~= 1 % we only care about walls
        continue
    end
    if node(1) == col && node(2) == row % collission check
        result  = true;
        return
    end
  end
end

result = false;

    