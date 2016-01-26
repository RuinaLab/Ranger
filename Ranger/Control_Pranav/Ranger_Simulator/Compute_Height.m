function [Heel, Hip] = Compute_Height(Y,robot)

% FUNCTION: 
%   This function takes a set of states (individual columns of Y) and
%   computes the height of the swing leg heel at all of them.

l = robot.parm.l;
d = robot.parm.d;
r = robot.parm.r;

q1 = Y(1,:);
q2 = Y(3,:);
q3 = Y(7,:);
q4 = Y(9,:);

StanceLeg = -l*cos(q1+q2)+d*cos(q1);

%This function taken directly from Pranav's code in robotfn.m. It computes
%the height of the swing leg heel
Heel = StanceLeg+l*cos(-q3+q1+q2)-d*cos(q1+q2-q3-q4);

%This function is also from Pranav's code in robotfn. It computes the
%height of the hip.
Hip = StanceLeg + r;

end