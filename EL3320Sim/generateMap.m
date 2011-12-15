%
% Function that makes it easy to create landmarks for a map as well as 
% a trajectory for the robot to drive.
%
% Authors: Patric Jensfelt, 2008
%          Alper Aydemir, 2009
%

function posLandm=generateMap()

clf
addMore = 1;
posLandm = [];
trajectory = [];
disp('Left-click in figure to add a landmark')
disp('Mid/right-click in figure to add a way point in the trajectory')
disp('Press ENTER once to display what you got so far')
disp('Press ENTER twice to end input of landmarks')
while addMore
    axis([-10 10 -10 10].*2)
    [x y button] = ginput();
    if isempty(x)
        addMore = 0;
    else
        for i=1:length(x)
            if button(i) == 1
                posLandm = [posLandm; [x(i) y(i)] ];
            else
                trajectory = [trajectory; [x(i) y(i)] ];
            end
        end
    end
    hold on
    if ~isempty(posLandm)
        h = plot(posLandm(:,1), posLandm(:,2),'ro');
    end
    if ~isempty(trajectory)
         for i=1:length(trajectory)
             h = plot(trajectory(i,1), trajectory(i,2),'bx');
             str = strcat('\leftarrow   ', num2str(i)); 
             text(trajectory(i,1), trajectory(i,2),str);
             legend(h,'way points');
         end
    end
end

if ~isempty(posLandm)
    posLandm = [(1:size(posLandm,1))' posLandm];
    fid = fopen('map.txt', 'w');
    for k = 1:size(posLandm,1)
        fprintf(fid, '%d %.3f %.3f\n', posLandm(k,1), posLandm(k,2), posLandm(k,3));
    end
    fclose(fid);
end

if ~isempty(trajectory)
    fid = fopen('traj.txt', 'w');
    for k = 1:length(trajectory)
        fprintf(fid, '#Goto (%.3f,%.3f) with %.1fm tolerance\n', trajectory(k,1), trajectory(k,2),0.1);
        fprintf(fid, '%d %.3f %.3f %.3f\n\n', 1, trajectory(k,1), trajectory(k,2),0.1);
    end
    fclose(fid);
end

