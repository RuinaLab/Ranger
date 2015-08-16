% Ranger ankle joint parameter identification experiment
% Matthew Kelly
% August 15, 2015
%
% Procedure:
% - robot is hanging from the ceiling
% - ankle motor controllers are tracking a sine-curve reference
%


dataFileName = 'Data6.txt';

% Read data from the log file
channelList = [65,87,62,63,61,248,249];
data = readData(dataFileName,channelList);

% Extract useful data:
qa = getChannel(data,65,'Ankle Angle');
dqa = getChannel(data,87,'Ankle Rate');
u = getChannel(data,62,'Motor Current');
qm = getChannel(data,63,'Motor Angle');
dqm = getChannel(data,61,'Motor Rate');
qt = getChannel(data,248,'Target Angle');
dqt = getChannel(data,249,'Target Rate');





%% Plot the raw data
figure(1); clf;

subplot(3,1,1); hold on;
title('Angle data')
plot(qt.time, qt.data, 'k')
plot(qm.time, qm.data, 'b')
plot(qa.time, qa.data, 'r')
xlabel('time (sec)')
ylabel('angle (rad)')
legend('target','motor','ankle')

subplot(3,1,2); hold on;
plot(dqt.time, dqt.data, 'k')
plot(dqm.time, dqm.data, 'b')
plot(dqa.time, dqa.data, 'r')
title('Rate data')
xlabel('time (sec)')
ylabel('rate (rad/sec)')
legend('target','motor','ankle')

subplot(3,1,3); hold on;
plot(u.time, u.data,'b')
xlabel('time (sec)')
ylabel('current (amps)')


%% Fit a function to the motor angle, and then differentiate

qm = fitSineCurve(qm);

u = fitSineCurve(u);

subplot(3,1,3); hold on;
plot(qm.time, u.fun(qm.time),'g','LineWidth',1);


%% Fit a joint inertia:

Km = 0.612;  %Nm/amp;  Motor constant for ankles (including gear box)

torque = Km*u.data;
accel = qm.ddFun(u.time);
inertia = torque./accel;

% NOTES:
% 
% Note sure if this is right or not. Might need a better method. The
% inertia is not at all consistent throughout the trial, suggesting that
% there is something else going on, or that we really need to get better
% current data, which the robot doesn't really produce.
%
% At the very least, this should be run through the full motor model, and
% then fit using optimization. The various nonlinear effects in the motor
% probably play an important role.
%
% The median value for inertia is actually pretty close to that of the
% motor's rotor inertia in the paper, although I still want to investigate
% where the gearbox comes into play there.
%




