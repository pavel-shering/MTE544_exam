%% Rapidly Expanding Random Tree - RRT
clear; clc;

%% Create AVI object
make_video = 0;
if (make_video)
    vidObj = VideoWriter('prmIII.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 15;
    open(vidObj);
end

% Set up environment
clear all; close all; clc

rng(1)
posMinBound = [0 0];
posMaxBound = [50 40];
posR = posMaxBound - posMinBound;
numObsts = 70;
endPos = [1 1];
startPos = [48.5 38.5];

minLen.a = 1;
maxLen.a = 3;
minLen.b = 2;
maxLen.b = 6;

obstBuffer = 0.5;
maxCount = 10000;

[aObsts,bObsts,obsPtsStore] = polygonal_world(posMinBound, posMaxBound, ... 
    minLen, maxLen, numObsts, startPos, endPos, obstBuffer, maxCount);

% plotEnvironment(obsPtsStore,posMinBound, posMaxBound, startPos, endPos, 1);

figure(1); clf;
hold on;
plotEnvironment(obsPtsStore,posMinBound, posMaxBound, startPos, endPos, 1);

% % Set up the map
xMax = posMaxBound; % State bounds
xMin = posMinBound;
xR = posR;

% Set up the goals
x0 = [48.5 38.5];
xF = [1 1];

% % Set up the obstacles
% rand('state', 1);
nO = numObsts; % number of obstacles
nE = 4; % number of edges per obstacle (not changeable).

% % Define single freespace nonconvex polygon by its vertices, each hole
% % separated by NaNs
env = [xMin(1) xMin(2);xMin(1) xMax(2);xMax(1) xMax(2);xMax(1) xMin(2); xMin(1) xMin(2)]
obsEdges = [];
figure(1); hold on;
for i=1:nO
    env = [env; NaN NaN; obsPtsStore(:,2*(i-1)+1:2*i);obsPtsStore(1,2*(i-1)+1:2*i)];
    obsEdges = [obsEdges; obsPtsStore(1:nE,2*(i-1)+1:2*i) obsPtsStore([2:nE 1],2*(i-1)+1:2*i)];
end

% Plot obstacles
% figure(1); clf; hold on;
% plotEnvironment(obsPtsStore,xMin, xMax, x0, xF);
drawnow();
figure(1); hold on;
disp('Time to create environment');
toc;
% if (make_video) writeVideo(vidObj, getframe(gcf)); end


%% RRT, created until solution found
tic;
done = 0;
milestones = [x0 0; xR 0];
nM = 1;
t= 0;
max_iters = 1150;
collision_step_size = 0.1;

while ((~done) && (t < max_iters))
    t=t+1;
    % Select goal location
    % Uniform with growth factor - not in obstacle
    goal_found = false;
    while (~goal_found)
        growth_factor = 2*(t+50)/(max_iters+100)/1.5;
        cur_goal = (1-growth_factor)*xR + growth_factor * 2* x0.*rand(1,2);
        if (inpolygon(cur_goal(1), cur_goal(2), env(:,1), env(:,2)))
            goal_found = true;
        end
    end
   % plot(cur_goal(1),cur_goal(2), 'kx', 'MarkerSize', 6)
   
    % Find closest node 
    dist = zeros(1,length(milestones(:,1)));
    for i = 1:length(milestones(:,1))
        dist(i) = norm(cur_goal-milestones(i,1:2));
    end
    [maxdist, curstone] = min(dist);
    
    cur_edge = [milestones(curstone,1:2); cur_goal];
    %plot(cur_edge(:,1), cur_edge(:,2),'k','LineWidth',2);

    steps = floor(norm(cur_edge(1,:)-cur_edge(2,:))/collision_step_size);
    samples = milestones(curstone,1:2);
    for i=2:steps
        samples(i,:) = ((steps-i)/steps)*milestones(curstone,1:2) + (i/steps)*cur_goal;
    end
    %plot(samples(:,1), samples(:,2),'g','LineWidth',2);
    
    keep = inpolygon(samples(:,1), samples(:,2), env(:,1),env(:,2));

    if (sum(keep)==steps)
        milestones = [milestones; samples(end,:) curstone];
        nM = nM+1;
        plot(samples(:,1),samples(:,2),'m');
        plot(milestones(end,1),milestones(end,2),'mo');
    end
    
    % Check if a path from start to end is found
    last_edge = [xF;milestones(end,1:2)];
    steps = floor(norm(last_edge(1,:)-last_edge(2,:))/collision_step_size);
    samples = milestones(end,1:2);
    for i=2:steps
        samples(i,:) = ((steps-i)/steps)*milestones(end,1:2) + (i/steps)*xF;
    end
    keep = inpolygon(samples(:,1), samples(:,2), env(:,1),env(:,2));

    if (sum(keep)==steps)
        milestones = [milestones; samples(end,:) curstone];
        nM = nM+1;
        plot(samples(:,1),samples(:,2),'m');
        plot(milestones(end,1),milestones(end,2),'mo');
        done = 1;
    end
%     if (make_video) writeVideo(vidObj, getframe(gcf)); end
end

% Find and plot final path through back tracing
if (done)
    done = 0;
    cur = nM
    curC = milestones(nM,:);
    prev = curC(3);
    i=2;
    p=1;
    dtot= 0;
    nMiles = 0;
    while (~done)
        if (prev == 1)
            done = 1;
        end
        plot([milestones(prev,1) milestones(cur,1)], [milestones(prev,2)...
            milestones(cur,2)],'go','MarkerSize',6, 'LineWidth',2)
        dtot = dtot + norm(milestones(prev,1:2)-milestones(cur,1:2));
        nMiles = nMiles+1;
        cur = prev;
        curC = milestones(cur,:);
        prev = curC(3);
        p=p+1;
%         if (make_video) writeVideo(vidObj, getframe(gcf)); end
    end
    disp('Time to find a path');
    dtot
    toc;
else
    disp('No path found');
end
% if (make_video) close(vidObj); end

