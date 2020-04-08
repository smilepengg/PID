function robot = checkDC(robot)
% Makes sure robot doesn't stop moving, or exceed duty cycle upper limit
if robot.rMot > 1; robot.rMot=1; 
end
if robot.rMot < 0; robot.rMot=0;
end
if robot.lMot > 1; robot.lMot=1;
end
if robot.lMot < 0; robot.lMot=0;
end
end