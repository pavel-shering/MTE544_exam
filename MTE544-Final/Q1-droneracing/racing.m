% Aerial racing
clear all; clc;

% %% Create AVI object
% makemovie1 = 0; % Inverse Measurement Model Video
% if(makemovie1)
%     vidObj1 = VideoWriter('ex1_measurement_model.avi');
%     vidObj1.Quality = 100;
%     vidObj1.FrameRate = 4;
%     open(vidObj1);
% end
% makemovie2 = 0; % Occupancy Grid Video
% if (makemovie2)
%     vidObj2 = VideoWriter('ex1_occupancy_grid.avi');
%     vidObj2.Quality = 100;
%     vidObj2.FrameRate = 4;
%     open(vidObj2);
% end

I = imread('Racecourse.png');
map = im2bw(I, 0.4); % Convert to 0-1 image
map = flipud(1-map)'; % Convert to 0 free, 1 occupied and flip.
[M,N]= size(map); % Map size [876 x 676]

% Robot start position
dxy = 0.1;
startpos = dxy*[350 250];
checkpoints = dxy*[440 620; 440 665];


% waypoints = [400 50];
waypoints = [400  70; 700 100; 850 270;
             820 390; 710 390; 700 390;
             760 300; 730 210; 700 130; 600 180];
waypoint_index = 1;
dist_threshold = 15;

% % Plotting
% figure(1); clf; hold on;
% colormap('gray');
% imagesc(1-map');
% plot(startpos(1)/dxy, startpos(2)/dxy, 'ro', 'MarkerSize',10, 'LineWidth', 3);
% plot(checkpoints(:,1)/dxy, checkpoints(:,2)/dxy, 'g-x', 'MarkerSize',10, 'LineWidth', 3 );
% xlabel('North (decimeters)')
% ylabel('East (decimeters)')
% axis equal

% sensor properties
phi_m = -0.602:0.01:0.602; % between 69/2 degrees
r_max = 10/dxy/2; % Max range 10
r_min = 0.3/dxy;
alpha = 1; % Width of an obstacle (Distance about measurement to fill in)
beta = 0.05; % Width of a beam (Angle beyond which to exclude)
%Probabilites of cells
p_occ = 0.7;
p_free = 0.3;

% Simulation time
Tmax = 400;
T = 0:Tmax;

% State Initialization (x, y, theta)
% x,y position
% theta robot heading (absolute)
x0 = [350 250 0]';
x = zeros(3, length(T) + 1);
x(:, 1) = x0;

% input
u = [20 00 0]'; % [m/s m/s rad] [20 20 0]';

% Occupancy grid in both probablity and log odds form
og = 0.5*ones(M, N);
ogl0 = log(og./(1-og));
ogl = ogl0;
dt = 1/5; % update rate
%% Main simulation
for t = 2:length(T)+1
    %% Simulation

    % Motion Noise
    d = [normrnd(0,0.05); normrnd(0,0.05); normrnd(0,0.02)];
    
    % Robot motion
    [x(:,t)] = motion_model(x(:,t-1), u, dt); %+ d;

    % Generate a measurement data set
    [r_m not_these] = getranges(map, x(:,t), phi_m, r_max, alpha);
    
    %% Map update;
	% Call occupancy grid mapping logit update function using Bresenham
    [ogl, imml] = ogmap_update(ogl, x(:,t), phi_m, r_m, r_max, r_min, alpha, beta, p_occ, p_free, 0);

    % Calculate probability map from log odds map
    og = exp(ogl)./(1 + exp(ogl));
    og_mm = exp(imml)./(1 + exp(imml));
    
    %% Plot results
    % Map and vehicle path
    figure(1); clf; hold on;
    colormap('gray');
    imagesc(1-map');
    plot(startpos(1)/dxy, startpos(2)/dxy, 'ro', 'MarkerSize',10, 'LineWidth', 3);
    plot(checkpoints(:,1)/dxy, checkpoints(:,2)/dxy, 'g-x', 'MarkerSize',10, 'LineWidth', 3 );
    plot(x(1, 1:t), x(2, 1:t), 'bx-')
%     drawbot(x(1, 1:t), x(2, 1:t), x(3, 1:t), 4, 1)
    xlabel('North (decimeters)')
    ylabel('East (decimeters)')
    axis([0 M 0 N])
    F1(t-1) = getframe(gcf);
    
    % Inverse measurement model
    figure(2);clf; hold on;
    image(100*(1-og_mm)');
    colormap(gray);
    plot(x(1,t)/dxy,x(2,t)/dxy,'bx')
    axis([0 M 0 N])
    title('Measurements and inverse measurement model');
    F2(t-1) = getframe(gcf);
    
    % Belief map
    figure(3);clf; hold on;
    image(100*(1-og)');
    colormap(gray);
    axis([0 M 0 N])
    title('Current occupancy grid map')
    F3(t-1) = getframe(gcf);

    u = traj(waypoints(waypoint_index,:), x(:,t), u, r_m, phi_m, not_these)';
    a = x(1:2,t) - waypoints(waypoint_index,:)'
    if (abs(x(1:2,t) - waypoints(waypoint_index,:)') < dist_threshold)
        disp('done')
        waypoint_index = waypoint_index+1;
    end

end
