function robot = newPos(robot,dT,i,filter_enable)

dir_old = robot.dir(:,i-1); % Direction of robot at start of step

lDC = robot.lMot*1.02; % Left and right motor duty cycle-->left motor is 2% stronger
rDC = robot.rMot; 
wMax = robot.wMax; % Max angular velocity of motors

vL=(wMax*lDC*2*pi*robot.r); % Tangential speed of left and right wheels (m/s) 
vR=(wMax*rDC*2*pi*robot.r);

dL=(vL*dT*dir_old)*my_noise+robot.lWheel(:,i-1); % Intended positions of left and right wheels
dR=(vR*dT*dir_old)*my_noise+robot.rWheel(:,i-1);

robot.path(:,i)=mean([dL dR],2); % Update centroid of robot

diff=dL-dR; % Vector from right to left wheel

robot.lWheel(:,i) = robot.path(:,i)+diff/norm(diff)*robot.d/2; % Constrain wheels to be d apart from centroid
robot.rWheel(:,i) = robot.path(:,i)-diff/norm(diff)*robot.d/2;

diff2=diff; diff2(1)=-diff(2); diff2(2)=diff(1); % New direction vector orthogonal to vector between wheels
robot.dir(:,i) = diff2/norm(diff2);

% The next section of code is the implementation of a filter which reduces
% the noise of encoder readings. To turn this setting on, the parameter
% filter_enable on line 18 of main_script.m must be set to 1. This will give 
% more accurate values to the PID control system. However computation time
% and overall simulation time will be significantly longer. Therefore, the
% filter should be used with caution.

if filter_enable==0
    % Simulate encoder distance readings, with noise
    robot.encL(i)=robot.encL(i-1)+norm(robot.lWheel(:,i)-robot.lWheel(:,i-1))*my_noise2;
    robot.encR(i)=robot.encR(i-1)+norm(robot.rWheel(:,i)-robot.rWheel(:,i-1))*my_noise2;

elseif filter_enable==1
    size_window=2;
    x=1;
    y=(1/size_window)*ones(1,size_window);
    
    % Simulate encoder distance readings, with noise
    robot.encL(i)=robot.encL(i-1)+norm(robot.lWheel(:,i)-robot.lWheel(:,i-1))*my_noise2;
    robot.encR(i)=robot.encR(i-1)+norm(robot.rWheel(:,i)-robot.rWheel(:,i-1))*my_noise2;
    
    % Applying the filter
    encL_filtered=filter(y,x,robot.encL);
    encR_filtered=filter(y,x,robot.encR);
   
    % New filtered encoder readings
    robot.encL=encL_filtered;
    robot.encR=encR_filtered;
end