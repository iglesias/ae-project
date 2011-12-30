%
% Function that makes it easy to create landmarks for a map as well as 
% several trajectories for the robots to drive.
%
% Authors: Patric Jensfelt, 2008
%          Alper Aydemir, 2009
%          Fernando J. Iglesias García, 2011

function posLandm=generateMap()

clf
addMore   = 1;
posLandm  = [];
trajs     = {};
trajs{1}  = [];
trajIdx   = 1;   % Index for the current trajectory

disp('Left-click in figure to add a landmark')
disp('Right-click in figure to add a way point in the trajectory')
disp('Mid-click in figure to start a new trajectory')
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
            elseif button(i) == 2
                trajIdx         = trajIdx + 1;
                trajs{trajIdx}  = [];
            elseif button(i) == 3
                trajs{trajIdx} = [trajs{trajIdx}; [x(i) y(i)] ];
            end
        end
    end
    hold on
    if ~isempty(posLandm)
        plot(posLandm(:,1), posLandm(:,2),'ro');
    end
    if ~isempty( trajs{trajIdx} )
         for i = 1:length( trajs{trajIdx} )
             h = plot(trajs{trajIdx}(i,1), trajs{trajIdx}(i,2),'bx');
             str = strcat('\leftarrow   ', num2str(i)); 
             text(trajs{trajIdx}(i,1), trajs{trajIdx}(i,2), str);
             legend(h, 'way points');
         end
    end
end

if ~isempty(posLandm)
    posLandm = [(1:size(posLandm,1))' posLandm];
    fid = fopen('map.txt', 'w');
    for k = 1:size(posLandm,1)
        fprintf(fid, '%d %.3f %.3f\n', ...
                posLandm(k,1), posLandm(k,2), posLandm(k,3));
    end
    fclose(fid);
end

if ~isempty(trajs)
    for i = 1:length(trajs)
        fid = fopen(['traj' sprintf('%d', i) '.txt'], 'w');
        for k = 1:length(trajs{i})
            fprintf(fid, '#Goto (%.3f,%.3f) with %.1fm tolerance\n', ...
                    trajs{i}(k, 1), trajs{i}(k, 2), 0.1);
            fprintf(fid, '%d %.3f %.3f %.3f\n\n', 1, ...
                    trajs{i}(k, 1), trajs{i}(k, 2), 0.1);
        end
        fclose(fid);
    end
end

