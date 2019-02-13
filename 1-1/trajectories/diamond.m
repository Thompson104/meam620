function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
% NOTE: the simulator will spawn the robot to be at the
%       position you return for t == 0
points=[0,0,0;1/4,sqrt(2),sqrt(2);1/2,0,2*sqrt(2);3/4,-sqrt(2),sqrt(2);1,0,0];
time2goal=11;
dthetadt=2*pi/time2goal;
[numSteps,~]=size(points);
numSteps=numSteps-1;
timePerStep=time2goal/numSteps;

if t>time2goal
    x_des=points(end,1);
    y_des=points(end,2);
    z_des=points(end,3);
    x_vel=0;
    y_vel=0;
    z_vel=0;
    x_acc=0;
    y_acc=0;
    z_acc=0;
else
    index=ceil(t/timePerStep+1e-10);
    nextIndex=index+1;
    x_0=points(index,1);
    y_0=points(index,2);
    z_0=points(index,3);
    t_0=(index-1)*timePerStep;
    x_vel=(points(nextIndex,1)-points(index,1))/timePerStep;
    y_vel=(points(nextIndex,2)-points(index,2))/timePerStep;
    z_vel=(points(nextIndex,3)-points(index,3))/timePerStep; 
    x_des=x_vel*(t-t_0)+x_0;
    y_des=y_vel*(t-t_0)+y_0;
    z_des=z_vel*(t-t_0)+z_0;
    x_acc=0;
    y_acc=0;
    z_acc=0;
end
pos = [x_des; y_des; z_des];
vel = [x_vel;y_vel; z_vel];
acc = [x_acc;y_acc ; z_acc];
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
