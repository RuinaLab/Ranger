function [qr, stepLength] = getRobotAngle(qh,q0,q1,l,d, Phi, Slope)
% [qr, stepLength] = getRobotAngle(qh,q0,q1,l,d, Phi, Slope)
%
% This function computes the robot angle, given the joint angles of the
% robot and the slope of the ground.
%

% Assuming that the robot angle is vertical, find the vector from the
% virtual center of the outer feet tot the virtual center of the inner
% feet.
x =  l*sin(qh) - d*sin(Phi - q1 + qh) + d*sin(Phi - q0);
y = l + d*cos(Phi - q1 + qh) - l*cos(qh) - d*cos(Phi - q0);

% Now convert this vector to polar coordinates:
stepLength = sqrt(x.*x + y.*y);
angle = atan(y./x);

% Compensate for the ground angle:
qr = angle + Slope;

end