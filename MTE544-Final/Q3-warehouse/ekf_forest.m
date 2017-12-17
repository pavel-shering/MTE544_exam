%% ME 595 Final Exam - Fall 2009
% Forest definition for Question #2
close all; clear; clc;

trees = [ 16.6166   19.2380;
   10.9945   15.4982;
   18.3439   16.3461;
   15.1440    1.6887;
   15.0746    7.9957;
    1.0790   18.2130;
   18.6802    2.9108;
   11.3765   17.3858;
    6.7425    2.8991;
    3.2436   17.0606;
   15.8857   12.4411;
   10.5707   10.2650;
    3.3130    8.0362;
   12.0396    1.5193;
   13.7843    3.6782;
    9.0108    8.3453;
    4.5795   18.0543;
   16.5163    9.7851;
    1.5635    7.3849;
    8.8536    2.2241];

d = 1; % tree diameter

startpos = [1 2]; % Initial position
endpos = [15 16]; % Final position

%% Plot results
t=1;
figure(1);clf; hold on; % True map of forest
for i = 1:length(trees)
    % Yes, in Matlab, the rectangle function draws circles.
    rectangle('Position',[trees(i,1)-d/2,trees(i,2)-d/2,d,d], 'Curvature', [1 1], 'LineWidth',1, 'EdgeColor','b');
end
plot(startpos(1), startpos(2), 'gx', 'LineWidth', 2, 'MarkerSize', 6)
plot(endpos(1), endpos(2), 'rx', 'LineWidth', 2, 'MarkerSize', 6)
axis equal
axis([0 20 0 20]);

%% Simulation Initialization

% Time
Tf = 80;
dt = 0.5;
T = 0:dt:Tf;

% Initial Robot State
x0 = [1 0 2 0 2 0]';

% Motion model
A = [ 1 dt 0 0 0 0;
      0 1 0 0 0 0;
      0 0 1 dt 0 0;
      0 0 0 1 0 0;
      0 0 0 0 1 dt; 
      0 0 0 0 0 1];
B = [ 0 0 0;
      dt 0 0;
      0  0 0;
      0 dt 0;
      0 0 0;
      0 0 dt];

yaw = atan2(endpos(2)-startpos(2),endpos(1)-startpos(1));


% Motion Disturbance model
R = [0.001 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0.001 0 0 0; 
     0 0 0 0 0 0;
     0 0 0 0 0.001 0;
     0 0 0 0 0 0];
[RE, Re] = eig(R);

% Control inputs (initialization)
u = zeros(3, length(T));

% Prior over robot state
mu0r = [1 0 2 0 2 0]'; % mean (mu)
S0rr = 0.0000001*eye(6);% covariance (Sigma)

M = length(trees);
% Prior over feature map
mu0m = zeros(2*M,1);
S0mm = 100*eye(2*M);
newfeature = ones(M,1);

%Measurement model
rmax = 10; % Max range
thmax = 90*pi/180; % 1/2 Field of view

% Measurement noise
Qi = [0.001 0; 
     0 0.00001];

[QiE, Qie] = eig(Qi);

% Simulation Initializations
n = length(R(:,1)); % Number of vehicle states
xr = zeros(n,length(T)); % Vehicle states 
xr(:,1) = x0;
N = n+2*M;
m = length(Qi(:,1)); % Number of measurements per feature 
y = zeros(m*M,length(T)); % Measurements

mu = [mu0r; mu0m];
S = [S0rr zeros(n,2*M);  zeros(2*M,n) S0mm];
%S = [S0rr 100*ones(n,2*M);  100*ones(2*M,n) S0mm];

mu_S = zeros(N,length(T)); % Belief
mu_S(:,1) = mu;

% Control Gains
Katt = 0.1;
Krep = 10;
rc0 = 5;
Kdx = 1;
Kdy = 1;
Kpz = 1;
Kdz = 1;
zdes = 2;   
vdes = 0.3;

%% Main loop
for t=2:length(T)
    %% Simulation
    % Use potential fields to define an input u.
    curpos = mu(1:2:3); % x,y position
    % Attractive potential gradient to end point
    gVcur = Katt*(curpos - endpos');
    % Repulsive potential from trees
    for i=1:M
        if (~newfeature(i))
            curtree = mu(6+2*(i-1)+1:6+2*i);
            if (norm(curpos-curtree)<=d/2)
                gVcur = [NaN NaN];
            else
                centD = norm(curpos-curtree);
                if (centD < rc0)
                    gVcur = gVcur + Krep*(-1/centD+1/rc0)*(curpos-curtree)/centD^(3);
                end
            end
        end
    end
    
    phides = atan2(-gVcur(2),-gVcur(1));
    
    u(:,t) = [ Kdx*(vdes*cos(phides) - mu(2));
               Kdy*(vdes*sin(phides) - mu(4));
               Kpz*(zdes - mu(5)) + Kdz*(-mu(6))];

   
           
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update robot state4
    xr(:,t) = A*xr(:,t-1) + B*u(:,t) + e;
    % Yaw (heading of sensor) is held fixed throughout.
    
    % Take measurements
    % For each feature
    flist = zeros(M,1);
    for i=1:M
        % If feature is visible
        if (inview(trees(i,:),[xr(1,t) xr(3,t),yaw],rmax,thmax))
            flist(i) = 1;
            % Select a motion disturbance
            d = QiE*sqrt(Qie)*randn(m,1);
            % Determine measurement
            y(2*(i-1)+1:2*i,t) = [sqrt((trees(i,1)-xr(1,t))^2 + (trees(i,2)-xr(3,t))^2 + (-xr(5,t))^2);
                atan2(trees(i,2)-xr(3,t),trees(i,1)-xr(1,t))-yaw] + d;
        end
    end

    %% Extended Kalman Filter Estimation
    % Prediction update
    mu(1:n) = A*mu(1:n) + B*u(:,t);
    S(1:n,1:n) = A*S(1:n,1:n)*A' + R;

    % Measurement update
    for i=1:M
        if (flist(i))
            % Feature initialization
            if (newfeature(i) == 1)
                rxy = sqrt(y(2*(i-1)+1,t)^2-mu(5)^2);
                mu(6+2*(i-1)+1) = mu(1)+rxy*cos(y(2*i,t)+yaw);
                mu(6+2*i) = mu(3)+rxy*sin(y(2*i,t)+yaw);
                newfeature(i) = 0;
            end
            % Linearization
            % Predicted range
            dx = mu(6+2*(i-1)+1)-mu(1);
            dy = mu(6+2*i)-mu(3);
            dz = -mu(5);
            rp = sqrt((dx)^2+(dy)^2+ (dz)^2);
            rxy = sqrt(rp^2 - dz^2);
            Fi = zeros(5,N);
            Fi(1:n,1:n) = eye(n);
            Fi(7:8,6+2*(i-1)+1:6+2*i) = eye(2);
            Ht = [ -dx/rp, 0, -dy/rp, 0, -dz/rp, 0, dx/rp, dy/rp;
                dy/rxy^2, 0, -dx/rxy^2, 0, 0, 0, -dy/rxy^2, dx/rxy^2]*Fi;

            I = y(2*(i-1)+1:2*i,t)-[rp;
                (atan2(dy,dx) - yaw)];
 
            % Measurement update
            K = S*Ht'/(Ht*S*Ht'+Qi);
            mu = mu + K*I;
            S = (eye(n+2*M)-K*Ht)*S;
        end
    end
 
    % Store results
    mu_S(:,t) = mu;


    %% Plot results
    figure(1);
    %subplot(1,2,1); hold on;
    %plot(trees(:,1),trees(:,2),'go', 'MarkerSize',10,'LineWidth',2);
    plot(xr(1,1:t),xr(3,1:t), 'ro--')
    %plot([xr(1,t) xr(1,t)+1*cos(yaw)],[xr(3,t) xr(3,t)+1*sin(yaw)], 'r-')
    plot(mu_S(1,1:t),mu_S(3,1:t), 'bx--')
    %plot([mu_S(1,t) mu_S(1,t)+1*cos(yaw)],[mu_S(3,t) mu_S(3,t)+1*sin(yaw)], 'b-')
    mu_pos = [mu(1) mu(3)];
    S_pos = [S(1,1) S(1,3); S(3,1) S(3,3)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    plot( [mu(1) mu(1)+2*cos(phides)],[mu(3) mu(3)+2*sin(phides)], 'b')
    for i=1:M
          if (flist(i))
              fi = 2*(i-1)+1;
              fj = 2*i;
              rxy = sqrt(y(fi,t)^2-xr(5,t)^2);
              plot([mu(1) mu(1)+rxy*cos(y(fj,t)+yaw)], [mu(3) mu(3)+rxy*sin(y(fj,t)+yaw)], 'c');
              plot(mu(6+fi),mu(6+fj), 'gx')
              mu_pos = [mu(6+fi) mu(6+fj)];
              S_pos = [S(6+fi,6+fi) S(6+fi,6+fj); S(6+fj,6+fi) S(6+fj,6+fj)];
              error_ellipse(S_pos,mu_pos,0.95);
          end
    end
    axis equal
%     axis([-4 6 -1 7])
    title('SLAM with Range & Bearing Measurements')
    
    %subplot(1,2,2);
    %image(10000*S);
    %colormap('gray');
    %title('Covariance matrix')
 
   % F(t) = getframe(gcf);
end

