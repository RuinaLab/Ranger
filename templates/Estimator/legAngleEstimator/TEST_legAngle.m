% TEST_legAngle.m
%
% This script tests to code that figures out the robot orientation when in
% double stance.
%

% Specify the absolute orientation of each link of the robot:

l = 0.96;  % Leg length
d = 0.14;  % Foot eccentricity
Phi = 1.8;   %Nominally "flat" foot angle

%%%% A few sanity checks:
Slope = [0.0, 0.0, 0.0, 0.1, 0.0];
qh = [-0.3, -0.4, 0.3, 0.3, 0.1];   % negative --> Outer leg in front
q0 = [1.95, 2.0, 1.65, 1.65, 1.6];
q1 = [1.65, 1.6, 1.95, 1.95, 2.0];

qr = [-0.15, -0.2, 0.15, 0.25, 0.05];

[qRobot, stepLength] = getRobotAngle(qh,q0,q1,l,d, Phi, Slope);

checkMatch = [qr', qRobot'];

