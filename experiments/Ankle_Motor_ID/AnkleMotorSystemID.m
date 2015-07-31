% Ranger ankle motor parameter identification experiment
% Matthew Kelly
% July 28, 2015
%
% Procedure:
% - robot is hanging from the ceiling
% - every 0.75 seconds a new random target angle command is sent to the
% tracking controller for each ankle.
% - commands are from a uniform distribution [0.2, 2.0], the usable range
% of the ankle joints
% - Data was logged for both ankles.
%


error('This code does not work. See comments below.')
%%%% NOTES
%
% I'm not getting very good results from this data analysis. I think
% that the problem is that all of my input tests were step functions, and
% also that my model-fitting is based on acceleration matching from
% numerical differentiation. 
%
% Perhaps I would have better luck if I tried again, but used some sort of
% sine wave trajectory tracking, not step functions. This should produce
% more usable data, although it remains to be seen if this method is
% actually good for this type of system identification.
%
%%%%



% Read data from the log file
DATA = dlmread('DataLog.txt','\t');
ID = DATA(:,1);
% VALUE = DATA(:,2);
% TIMESTAMP = DATA(:,3);

% Convert to seconds:
DATA(:,3) = DATA(:,3)/1000;

%% Extract only the useful data
ID_u0 = 35;   %outer ankle current
ID_q0 = 37;   %outer ankle angle
ID_dq0 = 59;  %outer ankle rate
ID_u1 = 62;  %inner ankle current
ID_q1 = 65;  %inner ankle angle
ID_dq1 = 87;  %inner ankle rate

u0_raw = DATA(ID==ID_u0, [3,2]);
q0_raw = DATA(ID==ID_q0, [3,2]);
dq0_raw = DATA(ID==ID_dq0, [3,2]);
u1_raw = DATA(ID==ID_u1, [3,2]);
q1_raw = DATA(ID==ID_q1, [3,2]);
dq1_raw = DATA(ID==ID_dq1, [3,2]);

% Remove duplicate enteries:
idxRm = diff(u0_raw(:,1))==0; u0 = u0_raw(~idxRm,:);
idxRm = diff(u1_raw(:,1))==0; u1 = u1_raw(~idxRm,:);
idxRm = diff(q0_raw(:,1))==0; q0 = q0_raw(~idxRm,:);
idxRm = diff(q1_raw(:,1))==0; q1 = q1_raw(~idxRm,:);
idxRm = diff(dq0_raw(:,1))==0; dq0 = dq0_raw(~idxRm,:);
idxRm = diff(dq1_raw(:,1))==0; dq1 = dq1_raw(~idxRm,:);

%% Plot the raw data
figure(1); clf;

subplot(3,1,1); hold on;
plot(u0(:,1),u0(:,2))
plot(u1(:,1),u1(:,2))
legend('outer','inner')
title('Ankle Motor Raw Data')
xlabel('time (sec)')
ylabel('current (amp)')

subplot(3,1,2); hold on;
plot(q0(:,1),q0(:,2))
plot(q1(:,1),q1(:,2))
legend('outer','inner')
xlabel('time (sec)')
ylabel('angle (rad)')

subplot(3,1,3); hold on;
plot(dq0(:,1),dq0(:,2))
plot(dq1(:,1),dq1(:,2))
legend('outer','inner')
xlabel('time (sec)')
ylabel('rate (rad/s)')



%% Uniformly sample the data for filtering

% Find the maximum viable time window:
t0 = max([min(u0(:,1)),min(u1(:,1)),min(q0(:,1)),min(q1(:,1))]);
tF = min([max(u0(:,1)),max(u1(:,1)),max(q0(:,1)),max(q1(:,1))]);
dt = mean([diff(u0(:,1));diff(u1(:,1));diff(q0(:,1));diff(q1(:,1))]);
DT = dt/4; %Oversample to catch more peaks.
T = (t0:DT:tF)';

% Linear interpolate to a uniform grid:
U0 = interp1(u0(:,1), u0(:,2), T);
U1 = interp1(u1(:,1), u1(:,2), T);
Q0 = interp1(q0(:,1), q0(:,2), T);
Q1 = interp1(q1(:,1), q1(:,2), T);

% Create a butterworth smoothing filter
freqCutoff = 0.5*(1/dt);  %Filter cut-off frequency
freqSample = 1/DT;  %Interpolated data sample frequency
wn = freqCutoff/(0.5*freqSample);
[B, A] = butter(4,wn);

% Smooth the data:
U0 = filtfilt(B, A, U0);
U1 = filtfilt(B, A, U1);
Q0 = filtfilt(B, A, Q0);
Q1 = filtfilt(B, A, Q1);

%% Numerical differentiation to find the rate based on smoothed angle

dQ0 = diffCenter(Q0,DT);
dQ1 = diffCenter(Q1,DT);
ddQ0 = diffCenter(dQ0,DT);
ddQ1 = diffCenter(dQ1,DT);


%% Sample the data at twice the smoothing cutoff frequency:

InputPeriod = 0.75*60;   %Full period of the random motor commands
t0 = mean(T) - 0.5*InputPeriod;
tF = t0 + InputPeriod;
dt = 1/(2*freqCutoff);

t = (t0:dt:tF)';

u0 = interp1(T,U0,t,'pchip');
u1 = interp1(T,U1,t,'pchip');
q0 = interp1(T,Q0,t,'pchip');
q1 = interp1(T,Q1,t,'pchip');
dq0 = interp1(T,dQ0,t,'pchip');
dq1 = interp1(T,dQ1,t,'pchip');
ddq0 = interp1(T,ddQ0,t,'pchip');
ddq1 = interp1(T,ddQ1,t,'pchip');


%% Plot the nice smooth data for analysis
figure(2); clf;

subplot(4,1,1); hold on;
plot(t,u0)
plot(t,u1)
legend('outer','inner')
title('Ankle Motor Processed Data')
xlabel('time (sec)')
ylabel('current (amp)')

subplot(4,1,2); hold on;
plot(t,q0)
plot(t,q1)
legend('outer','inner')
xlabel('time (sec)')
ylabel('angle (rad)')

subplot(4,1,3); hold on;
plot(t,dq0)
plot(t,dq1)
legend('outer','inner')
xlabel('time (sec)')
ylabel('rate (rad/s)')

subplot(4,1,4); hold on;
plot(t,ddq0)
plot(t,ddq1)
legend('outer','inner')
xlabel('time (sec)')
ylabel('accle (rad/s)')

%% Run non-linear least squares to fit a motor model:

[MotorModel0, Info0] = fitAnkleMotorModel(u0,q0,dq0,ddq0);
[MotorModel1, Info1] = fitAnkleMotorModel(u1,q1,dq1,ddq1);

ddq0_model = Info0.ddqModel;
ddq1_model = Info1.ddqModel;

figure(3); clf;

subplot(2,1,1); hold on;
plot(t,ddq0)
plot(t,ddq0_model);
legend('data','model');
title('Model Fit, Outer Feet');
xlabel('time (sec)')
ylabel('accel (rad/sec^2)')

subplot(2,1,2); hold on;
plot(t,ddq1)
plot(t,ddq1_model);
legend('data','model');
title('Model Fit, Inner Feet');
xlabel('time (sec)')
ylabel('accel (rad/sec^2)')








