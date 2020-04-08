
% 0 = no control; 1 = proportional control; 2=PID control; 3=PID+compass error control; 
% 4=Integral control only; 5=Derivative control only;
% 6=PI control; 7=PD control; 8=Integral+compass control

function robot = drive_robot(robot,T,dT,ctrl_enable,filter_enable)

% Initializing
DC_ideal=getDC(robot.setSpeed,robot.r,robot.wMax); % Get duty cycle to initialize motors
robot.rMot = DC_ideal;
robot.lMot = DC_ideal;

% Working out timings
Tstep=T(2); % Step size of simulation
ctrl_interval=round(dT/Tstep);

% Control parameters *** Add parameters here as necessary ***
Kp=180; % Proportional control parameter
Ki=1.8; % Integral control parameter
Kd=0.1; % Derivative control parametr

figure;

for i=2:length(T)
    
    % Compute change in position
    robot=newPos(robot,Tstep,i,filter_enable);
    
    % Compass Reading instead of Encoder Reading
    robot.heading=atan((robot.dir(2,i))/(robot.dir(1,i)));
    
    if mod(i,ctrl_interval)< eps %If it is time to provide feedback
       switch ctrl_enable % Open loop control; no feedback
           case 0; % Open loop, do nothing
           case 1; % Proportional control
               
               % Get error signal (Computed as Left - Right encoder reading)
               enc_error = getError(robot,i,ctrl_interval);
               % Modify motor duty cycle accordingly
               robot.rMot = robot.rMot+Kp*enc_error;
               robot.lMot = robot.lMot-Kp*enc_error;
               
               % Make sure PWM is still between [0, 1]
               robot=checkDC(robot);
               
           case 2; % PID (encoder error) control
               % Get error signal
               enc_error = getError(robot, i, ctrl_interval);
               
               % Proportional control
               p_ctrl=Kp*enc_error;
               
               % Integral control
               robot.error_tot=robot.error_tot+(enc_error); %summing all the errors
               i_ctrl=Ki*robot.error_tot;

               % Derivative control
               error_slope=(enc_error-robot.error_prev)/dT; %taking the derivative of the error
               d_ctrl=Kd*error_slope;
               
               % Adjusting the whole system/output of motors
               robot.rMot = robot.rMot+p_ctrl+i_ctrl+d_ctrl;
               robot.lMot = robot.lMot-p_ctrl-i_ctrl-d_ctrl; 
               
               % Storing present values
               robot.error_prev=enc_error;
               
          case 3; % PID + compass error (instead of encoder error) control
               % Get error signal
               enc_error = getError(robot, i, ctrl_interval);
               
               % Proportional control
               p_ctrl=Kp*robot.heading;
               
               % Integral control             
               robot.error_tot=robot.error_tot+(robot.heading); %summing all the errors
               i_ctrl=Ki*robot.error_tot;
               
               % Derivative control
               error_slope=(robot.heading-robot.error_prev)/dT; %taking the derivative of the error
               d_ctrl=Kd*error_slope;
               
               % Adjusting the whole system/output of motors
               robot.rMot = robot.rMot+p_ctrl+i_ctrl+d_ctrl;
               robot.lMot = robot.lMot-p_ctrl-i_ctrl-d_ctrl; 
               
               % Storing present values
               robot.error_prev=enc_error;
           
           case 4; % Integral control only
               % Get error signal
               enc_error = getError(robot, i, ctrl_interval);
               
               % Integral control
               robot.error_tot=robot.error_tot+(enc_error); %summing all the errors
               i_ctrl=Ki*robot.error_tot;
               
               % Adjusting the whole system/output of motors
               robot.rMot = robot.rMot+i_ctrl;
               robot.lMot = robot.lMot-i_ctrl;
               
            case 5; % Derivative control only
               % Get error signal
               enc_error = getError(robot, i, ctrl_interval);
               
               % Derivative control
               error_slope=(enc_error-robot.error_prev)/dT; %taking the derivative of the error
               d_ctrl=Kd*error_slope;
               
               % Adjusting the whole system/output of motors
               robot.rMot = robot.rMot+d_ctrl;
               robot.lMot = robot.lMot-d_ctrl;
               
               % Storing present values
               robot.error_prev=enc_error;
             
            case 6; % Proportional & Integral controls
               % Get error signal
               enc_error = getError(robot, i, ctrl_interval);
               
               % Proportional control
               p_ctrl=Kp*enc_error;
               
               % Integral control
               robot.error_tot=robot.error_tot+(enc_error); %summing all the errors
               i_ctrl=Ki*robot.error_tot;
               
               % Adjusting the whole system/output of motors
               robot.rMot = robot.rMot+p_ctrl+i_ctrl;
               robot.lMot = robot.lMot-p_ctrl-i_ctrl;
               
            case 7; % Proportional & Derivative controls
               % Get error signal
               enc_error = getError(robot, i, ctrl_interval);
               
               % Proportional control
               p_ctrl=Kp*enc_error;
               
               % Derivative control
               error_slope=(enc_error-robot.error_prev)/dT; %taking the derivative of the error
               d_ctrl=Kd*error_slope;
               
               % Adjusting the whole system/output of motors
               robot.rMot = robot.rMot+p_ctrl+d_ctrl;
               robot.lMot = robot.lMot-p_ctrl-d_ctrl;
               
               % Storing present values
               robot.error_prev=enc_error;
               
            case 8; % Integral + Compass control
               compass=Kc*robot.heading;
               
               % Get error signal
               enc_error = getError(robot, i, ctrl_interval);
               
               % Integral control
               robot.error_tot=robot.error_tot+(enc_error); %summing all the errors
               i_ctrl=Ki*robot.error_tot;
               
               % Adjusting the whole system/out of motors
               robot.rMot = robot.rMot+compass+i_ctrl;
               robot.lMot = robot.lMot-compass-i_ctrl;
           
       end
       
           % Code to visualize robot as it steps
%     clf; hold on;
%     scatter(robot.path(1,i),robot.path(2,i),'b');
%     scatter(robot.lWheel(1,i),robot.lWheel(2,i),'k');
%     scatter(robot.rWheel(1,i),robot.rWheel(2,i),'k');
%     axis tight
%     plot(xlim, [0 0], '-r')
%     xlim([-0.1 1]); ylim([-.2 .2]);
       
    end

end
end
