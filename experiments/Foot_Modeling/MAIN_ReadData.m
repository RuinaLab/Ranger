% Ranger ankle joint parameter identification experiment
% Matthew Kelly
% August 15, 2015
%
% Procedure:
% - robot is hanging from the ceiling
% - ankle motor controllers are tracking a sine-curve reference
%


dataFileName = 'Data2.txt';

% Read data from the log file
channelList = [65,87,62,63,61,248,249];
data = readData(dataFileName,channelList);

% Extract useful data:
qa = fitSineCurve(getChannel(data,65,'Ankle Angle'));
dqa = fitSineCurve(getChannel(data,87,'Ankle Rate'));
u = fitSineCurve(getChannel(data,62,'Motor Current'));
qm = fitSineCurve(getChannel(data,63,'Motor Angle'));
dqm = fitSineCurve(getChannel(data,61,'Motor Rate'));
qt = fitSineCurve(getChannel(data,248,'Target Angle'));
dqt = fitSineCurve(getChannel(data,249,'Target Rate'));


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
plot(qm.time, u.fun(qm.time),'g','LineWidth',2);
xlabel('time (sec)')
ylabel('current (amps)')


%% Compute a second-order system to match the data:
[m,c,k] = fitSecondOrderSys(qm,u);
