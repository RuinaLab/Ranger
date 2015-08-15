% Leg Angle Estimator
%
% This script solves the geometry problem of computing the orientation of
% the robot, given relative joint angles, and the fact that both feet are
% on the ground of a known slope.
%


clc; clear;


syms qh q0 q1 'real'  %Relative joint angles
syms PHI 'real'  %constant term in conversion between relative and absolute angles
qr = sym(0);  % Start by assuming robot angle is zero

% Convert to absolute coordinates:
th0 = -qr;
th1 = qh-qr;
phi0 = PHI-qr-q0;
phi1 = PHI+qh-qr-q1;

syms r d l 'real'  %Robot parameters
syms L 'real'  % Unknown step length

i = sym([1;0]);
j = sym([0;1]);


%%%% Unit vectors %%%%
ef0 = cos(phi0)*(-j) + sin(phi0)*(i); %Virtual center 0 to foot joint 0
ef1 = cos(phi1)*(-j) + sin(phi1)*(i); %Virtual center 1 to foot joint 1

el0 = cos(th0)*(-j) + sin(th0)*(i); %Hip to foot joint 0
el1 = cos(th1)*(-j) + sin(th1)*(i); %Hip to foot joint 1

%%%% Build out the kinematic tree:
pHip = [0;0];
p0 = pHip + l*el0;  % Outer ankle joint
p1 = pHip + l*el1;  % Inner ankle joint
p0c = p0 - d*ef0;  % outer ankle virtual center
p1c = p1 - d*ef1;  % inner ankle virtual center

%%%% Vector from one outer virtual center to inner:
delPc = p1c-p0c;

%%%% Next steps:
%
% 1) use delPc to get the vector from outer to inner feet
% 
% 2) the distance is the step length
% 
% 3) the angle of the vector is the sum of the robot angle and the ground
% slope
%






