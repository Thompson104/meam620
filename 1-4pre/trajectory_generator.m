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

time2goal=40;
[numSteps,~]=size(path);
numSteps=numSteps-1;
timePerStep=time2goal/numSteps;

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
    index=ceil(t/timePerStep+1e-10);
    nextIndex=index+1;
    x_0=path(index,1);
    y_0=path(index,2);
    z_0=path(index,3);
    t_0=(index-1)*timePerStep;
    x_vel=(path(nextIndex,1)-path(index,1))/timePerStep;
    y_vel=(path(nextIndex,2)-path(index,2))/timePerStep;
    z_vel=(path(nextIndex,3)-path(index,3))/timePerStep; 
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
