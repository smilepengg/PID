clear; close all; clc;
% Simulation parameters *** Edit these ***
N=20; % Number of seconds of robot movement to simulate
pps=1000; % Resolution of simulation (points per second)

% Control parameters *** Edit these ***
spd = 0.2; % Initial speed of robot (m/s)
dT = 0.01; % Time interval between controller changes (s)
% Note: The control interval must be a multiple of the simulation frequency.
ctrl_enable=2; % Enable control. 0 = no control; 1 = proportional control; 2=PID control; 3=PID+compass error control; 4=Integral control only; 5=Derivative control only; 6=PI control; 7=PD control; 8=Integral+compass control

% *Still* control parameters initialization
robot.error_tot=0;
robot.error_prev=0;

% Filter to reduce encoder noise
filter_enable=0; %1 to turn filter on

% Robot parameters *** Edit these ***
robot.r = 0.0485;    % wheel radius (m)
robot.wMax = 1.16;     % max motor angular velocity (rotations/second)
robot.d = 0.38;      % distance between wheels (m)

% Initalize simulation
robot.setSpeed = spd; % Set desired speed m/s
T=linspace(0,N,pps*N); % Create vector of time points
robot.path = zeros(2,pps*N); % x,y position of robot at all simulated time points
robot.dir = zeros(2,pps*N); robot.dir(:,1)=[1 0]; % robot starts off pointing in the x direction
robot.lWheel=zeros(2,pps*N); robot.lWheel(:,1) = [0; -robot.d/2]; % track wheel positions L / R
robot.rWheel=zeros(2,pps*N); robot.rWheel(:,1) = [0; +robot.d/2];
robot.encL=zeros(1,pps*N); robot.encR=zeros(1,pps*N); % Simulate encoder readings
robot.error=zeros(1,pps*N); % Track the error between the left and right encoder reading
robot.error=robot.encL-robot.encR;

% Run simulation 
robot=drive_robot(robot,T,dT,ctrl_enable, filter_enable);
robot.error=robot.encL-robot.encR;

% Plot and analyse results
figure;
hold on;
scatter(robot.path(1,:),robot.path(2,:),'b');
scatter(robot.lWheel(1,:),robot.lWheel(2,:),'k');
scatter(robot.rWheel(1,:),robot.rWheel(2,:),'k');
axis tight
plot(xlim, [0 0], '-r')
hold off;
title('Path travelled by robot');
xlabel('x (m)'); ylabel('y (m)');

% Plotting Error vs Time
figure;
hold on;
scatter(T,robot.error,'k');
axis tight
hold off;
title('Encoder Error Between Left and Right Encoder vs. Time');
xlabel('Time (s)'); ylabel('Error Between Left and Right Encoder (m)');