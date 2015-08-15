% Ranger ankle joint parameter identification experiment
% Matthew Kelly
% August 15, 2015
%
% Procedure:
% - robot is hanging from the ceiling
% - ankle motor controllers are tracking a sine-curve reference
%


dataFileName = 'Data1.txt';

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


% % 
% % %% Uniformly sample the data for filtering
% % 
% % % Find the maximum viable time window:
% % t0 = max([min(u0(:,1)),min(u1(:,1)),min(q0(:,1)),min(q1(:,1))]);
% % tF = min([max(u0(:,1)),max(u1(:,1)),max(q0(:,1)),max(q1(:,1))]);
% % dt = mean([diff(u0(:,1));diff(u1(:,1));diff(q0(:,1));diff(q1(:,1))]);
% % DT = dt/4; %Oversample to catch more peaks.
% % T = (t0:DT:tF)';
% % 
% % % Linear interpolate to a uniform grid:
% % U0 = interp1(u0(:,1), u0(:,2), T);
% % U1 = interp1(u1(:,1), u1(:,2), T);
% % Q0 = interp1(q0(:,1), q0(:,2), T);
% % Q1 = interp1(q1(:,1), q1(:,2), T);
% % 
% % % Create a butterworth smoothing filter
% % freqCutoff = 0.5*(1/dt);  %Filter cut-off frequency
% % freqSample = 1/DT;  %Interpolated data sample frequency
% % wn = freqCutoff/(0.5*freqSample);
% % [B, A] = butter(4,wn);
% % 
% % % Smooth the data:
% % U0 = filtfilt(B, A, U0);
% % U1 = filtfilt(B, A, U1);
% % Q0 = filtfilt(B, A, Q0);
% % Q1 = filtfilt(B, A, Q1);
% % 
% % %% Numerical differentiation to find the rate based on smoothed angle
% % 
% % dQ0 = diffCenter(Q0,DT);
% % dQ1 = diffCenter(Q1,DT);
% % ddQ0 = diffCenter(dQ0,DT);
% % ddQ1 = diffCenter(dQ1,DT);
% % 
% % 
% % %% Sample the data at twice the smoothing cutoff frequency:
% % 
% % InputPeriod = 0.75*60;   %Full period of the random motor commands
% % t0 = mean(T) - 0.5*InputPeriod;
% % tF = t0 + InputPeriod;
% % dt = 1/(2*freqCutoff);
% % 
% % t = (t0:dt:tF)';
% % 
% % u0 = interp1(T,U0,t,'pchip');
% % u1 = interp1(T,U1,t,'pchip');
% % q0 = interp1(T,Q0,t,'pchip');
% % q1 = interp1(T,Q1,t,'pchip');
% % dq0 = interp1(T,dQ0,t,'pchip');
% % dq1 = interp1(T,dQ1,t,'pchip');
% % ddq0 = interp1(T,ddQ0,t,'pchip');
% % ddq1 = interp1(T,ddQ1,t,'pchip');
% % 
% % 
% % %% Plot the nice smooth data for analysis
% % figure(2); clf;
% % 
% % subplot(4,1,1); hold on;
% % plot(t,u0)
% % plot(t,u1)
% % legend('outer','inner')
% % title('Ankle Motor Processed data')
% % xlabel('time (sec)')
% % ylabel('current (amp)')
% % 
% % subplot(4,1,2); hold on;
% % plot(t,q0)
% % plot(t,q1)
% % legend('outer','inner')
% % xlabel('time (sec)')
% % ylabel('angle (rad)')
% % 
% % subplot(4,1,3); hold on;
% % plot(t,dq0)
% % plot(t,dq1)
% % legend('outer','inner')
% % xlabel('time (sec)')
% % ylabel('rate (rad/s)')
% % 
% % subplot(4,1,4); hold on;
% % plot(t,ddq0)
% % plot(t,ddq1)
% % legend('outer','inner')
% % xlabel('time (sec)')
% % ylabel('accle (rad/s)')
% % 
% % %% Run non-linear least squares to fit a motor model:
% % 
% % [MotorModel0, Info0] = fitAnkleMotorModel(u0,q0,dq0,ddq0);
% % [MotorModel1, Info1] = fitAnkleMotorModel(u1,q1,dq1,ddq1);
% % 
% % ddq0_model = Info0.ddqModel;
% % ddq1_model = Info1.ddqModel;
% % 
% % figure(3); clf;
% % 
% % subplot(2,1,1); hold on;
% % plot(t,ddq0)
% % plot(t,ddq0_model);
% % legend('data','model');
% % title('Model Fit, Outer Feet');
% % xlabel('time (sec)')
% % ylabel('accel (rad/sec^2)')
% % 
% % subplot(2,1,2); hold on;
% % plot(t,ddq1)
% % plot(t,ddq1_model);
% % legend('data','model');
% % title('Model Fit, Inner Feet');
% % xlabel('time (sec)')
% % ylabel('accel (rad/sec^2)')
% % 
% % 
% % 
% % 
% % 
% % 
% % 
% % 
