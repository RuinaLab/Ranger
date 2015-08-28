% Ranger ankle joint parameter identification experiment
% Matthew Kelly
% August 15, 2015
%
% Procedure:
% - robot is hanging from the ceiling
% - ankle motor controllers are tracking a sine-curve reference
%


%%%% Something doesn't quite make sense with this data.  There is a weird
%%%% spike in the current data that seems strange. Need to investigate more
%%%% later this evening or tomorrow.

dataFileName = 'Data_TrackSine_Slow.txt';

% Read data from the log file
channelList = [242,243,65,73,74,75];
data = readData(dataFileName,channelList);

% Extract useful data:
qt = getChannel(data,242,'Target Angle');
dqt = getChannel(data,243,'Target Rate');
qm = getChannel(data,65,'Motor Angle');
Iref = getChannel(data,73,'CommandCurrent');
Cp = getChannel(data,74,'ProportionalGain');
Cd = getChannel(data,75,'DerivativeGain');

% Resample angles on a uniform grid:
t = linspace(qm.time(1),qm.time(end),length(qm.time));  %Resample to uniform grid
dt = (t(end)-t(1))/(length(t)-1);
Q = interp1(qm.time', qm.data',t','pchip')';

% Set up a butterworth filter:
fCutoff = 5;  %Cut-off information above this frequency
fNyquist = 0.5*(1/dt);
wn = fCutoff/fNyquist;
[B,A] = butter(2,wn);

% Compute derivatives and smoothing:
Q = filtfilt(B,A,Q);
dQ = filtfilt(B,A,diffCenter(Q,dt));
ddQ = filtfilt(B,A,diffCenter(dQ,dt));

% Compute command current:
cp = mean(Cp.data);  % Constant data
cd = mean(Cd.data);  % Constant data
U = interp1(Iref.time',Iref.data',t','pchip')' - cp*Q - cd*dQ;

%%%% Plot the data:
figure(2); clf;
% idx = 100:1800;   %Prevents matlab from crashing due to graphics failure
idx = 1:ceil(0.25*length(t));

subplot(3,1,1);
plot(t(idx),Q(idx));
xlabel('time')
ylabel('angle')
title('Ranger Inner Ankles')

subplot(3,1,2);
plot(t(idx),dQ(idx));
xlabel('time')
ylabel('rate')

subplot(3,1,3);
plot(t(idx),U(idx));
xlabel('time')
ylabel('current')


Alpha = 0.1;
Sgn = @(v)( tanh(v/Alpha) );  %Smooth sign of velocity
%%%% Fit a simple spring constant to the data:
%
% K(Q-Q0) + F*Sgn(dQ) = U
%
ONE = ones(size(Q));

aa = [Q', ONE', Sgn(dQ)'];
bb = U';
zz = aa\bb;

K = zz(1);
Q0 = -zz(2)/K;
F = zz(3);

U_spring = K*(Q-Q0) + F*Sgn(dQ);

subplot(3,1,3); hold on;
plot(t(idx),U_spring(idx));




