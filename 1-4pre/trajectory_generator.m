function [desired_state] = trajectory_generator(t, qn, varargin)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
% t: A scalar, specifying inquiry time
%
% varargin: variable number of input arguments. In the framework,
% this function will first (and only once!) be called like this:
%
% trajectory_generator([],[], 0, path)
%
% i.e. map = varargin{1} and path = varargin{2}.
%
% path: A N x 3 matrix where each row is (x, y, z) coordinate of a
% point in the path. N is the total number of points in the path
%
% This is when you compute and store the trajectory.
%
% Later it will be called with only t and qn as an argument, at
% which point you generate the desired state for point t.
%
persistent path
if nargin>2
    map = varargin{1};
end
if nargin>3
    path = varargin{2};
end

desired_state = [];

time2goal=20;
[numSteps,~]=size(path);
numSteps=numSteps-1;
timePerStep=time2goal/numSteps;

% path 
if t>time2goal
    x_des=path(end,1);
    y_des=path(end,2);
    z_des=path(end,3);
    x_vel=0;
    y_vel=0;
    z_vel=0;
    x_acc=0;
    y_acc=0;
    z_acc=0;
else
    currIndex=ceil(t/timePerStep+1e-10);
    nextIndex=currIndex+1;
    x_0=path(currIndex,1);
    y_0=path(currIndex,2);
    z_0=path(currIndex,3);
    t_0=(currIndex-1)*timePerStep;
    t_2=timePerStep+t_0;
    t_1=t_0+(t_2-t_0)/2;
    slopeVel_x=(path(nextIndex,1)-path(currIndex,1))/(t_1^2-t_0*t_1+t_0^2/2+t_2^2/2-t_1*t_2);
    slopeVel_y=(path(nextIndex,2)-path(currIndex,2))/(t_1^2-t_0*t_1+t_0^2/2+t_2^2/2-t_1*t_2);
    slopeVel_z=(path(nextIndex,3)-path(currIndex,3))/(t_1^2-t_0*t_1+t_0^2/2+t_2^2/2-t_1*t_2);
    if t<=t_1 %before the halfway point of this step
        x_vel=slopeVel_x*t-slopeVel_x*t_0;
        y_vel=slopeVel_y*t-slopeVel_y*t_0;
        z_vel=slopeVel_z*t-slopeVel_z*t_0; 
        x_des=slopeVel_x*(t^2/2-t_0*t+t_0^2/2)+x_0;
        y_des=slopeVel_y*(t^2/2-t_0*t+t_0^2/2)+y_0;
        z_des=slopeVel_z*(t^2/2-t_0*t+t_0^2/2)+z_0;
        x_acc=slopeVel_x;
        y_acc=slopeVel_y;
        z_acc=slopeVel_z;
    else
        x_vel=-slopeVel_x*t+slopeVel_x*t_2;
        y_vel=-slopeVel_y*t+slopeVel_y*t_2;
        z_vel=-slopeVel_z*t+slopeVel_z*t_2; 
        x_des=slopeVel_x*(t_1^2/2-t_0*t_1+t_0^2/2+t^2/2+t_1^2/2-t_1*t)+x_0;
        y_des=slopeVel_y*(t_1^2/2-t_0*t_1+t_0^2/2+t^2/2+t_1^2/2-t_1*t)+y_0;
        z_des=slopeVel_z*(t_1^2/2-t_0*t_1+t_0^2/2+t^2/2+t_1^2/2-t_1*t)+z_0;
        x_acc=-slopeVel_x;
        y_acc=-slopeVel_y;
        z_acc=-slopeVel_z;
    end
end
pos = [x_des; y_des; z_des];
vel = [x_vel;y_vel; z_vel];
acc = [x_acc;y_acc ; z_acc];
yaw = 0;
yawdot = 0;




% use the "persistent" keyword to keep your trajectory around
% inbetween function calls



%
% When called without varargin (isempty(varargin) == true), compute
% and return the desired state here.
%

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
