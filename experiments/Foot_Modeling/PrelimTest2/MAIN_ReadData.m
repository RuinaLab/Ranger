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

dataFileName = 'Data_FreqSweep.txt';

% Read data from the log file
channelList = [250,249,244,63,241,242,243];
data = readData(dataFileName,channelList);

% Extract useful data:
qt = getChannel(data,250,'Target Angle');
dqt = getChannel(data,249,'Target Rate');
period = getChannel(data,244,'Period');
qm = getChannel(data,63,'Motor Angle');
Iref = getChannel(data,241,'CommandCurrent');
Cp = getChannel(data,242,'ProportionalGain');
Cd = getChannel(data,243,'DerivativeGain');

% Resample angles on a uniform grid:
t = linspace(qm.time(1),qm.time(end),length(qm.time));  %Resample to uniform grid
dt = (t(end)-t(1))/(length(t)-1);
Q = interp1(qm.time', qm.data',t','pchip')';

% Set up a butterworth filter:
fCutoff = 10;  %Cut-off everything after 30 Hz.
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
idx = 100:800;   %Prevents matlab from crashing due to graphics failure

subplot(4,1,1);
plot(t(idx),Q(idx));

subplot(4,1,2);
plot(t(idx),dQ(idx));

subplot(4,1,3);
plot(t(idx),ddQ(idx));

subplot(4,1,4);
plot(t(idx),U(idx));

%%%% Fit a second order system:
[M,C,K,Q0] = fitSecondOrderSystem(ddQ,dQ,Q,U)

%%%% Fit damping:
% M = (1.16e-6)*34*34;   %Effective motor inertia
% [C,K,Q0] = fitSystem(ddQ,dQ,Q,U,M);

% This doesn't seem to work well either. 
%
% New plan: Use the effective mass of the rotor, and compute the spring
% parameters in stead-state with a very slow frequency sweep. Then do a
% single faster frequency sweed to get the damping.



