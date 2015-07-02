close all
clear

% Parameters
%P_Overhead = 4.5;   %(W) overhead electrical costs
%P_Camera = 0.1;   %(W) power used by the camera board
%disp('WARNING: CHECK CAMERA POWER')
%P_Steering = 0.5;  %(W) power for steering
%Distance_Traveled = 24.69;    %(meters)  Distance the robot walked  (HALLWAY KIMBALL)
%Distance_Traveled = 495.5*4/100;   %(meters) == 4 panels, 195" = 495.5cm each at Home Depot.

% disp('WARNING: Check distance traveled')
% disp(['Distance Travelled: ' num2str(Distance_Traveled)]);

%Use_Steering_Data = true;  %This integrates the steering motor power

MS_to_S = 0.001;   %Converts milliseconds to seconds
Weight = 9.91*9.81; %9.91kg*9.81m/s^2 = Newtons 

% SELECT THE LATEST FILE
% C:\Documents and Settings\Robot Admin\My
% Documents\SVN\Trunk\Data_Display\LabView_DAQ_Display\ROBO_DAQ_7

File_List = dir('Test*.txt');

FILE_NUMBER = 0;    % 0 = most recent, (N-1) = last file

fileName = File_List(end-FILE_NUMBER).name;
disp(['File Name: ' fileName])

% read data 
Raw_Data = textread(fileName); % B = [can_id , value , time] from the chosen file

% Parse the data by CAN_ID
Power_Hip = Raw_Data((Raw_Data(:,1)==15),2:3);
Power_Out = Raw_Data((Raw_Data(:,1)==44),2:3);
Power_Inn = Raw_Data((Raw_Data(:,1)==70),2:3);
Power_Steer = Raw_Data((Raw_Data(:,1)==113),2:3);

HS = Raw_Data((Raw_Data(:,1)==215),2:3);
RC = Raw_Data((Raw_Data(:,1)==385),2:3);    %ID_NAV_RC_USED = 385

MCU_Volt = Raw_Data((Raw_Data(:,1)==167),2:3);    %ID_CSR_MCU_VOLTAGE = 167, Power supply voltage
MCU_Curr = Raw_Data((Raw_Data(:,1)==168),2:3);    %ID_CSR_MCU_CURRENT = 168, Power supply current


%Get the total distance travelled
Dist_Tot = Raw_Data((Raw_Data(:,1)==207),2:3); % ID_E_TOTAL_DISTANCE = 207

% Plot the data in a useful way
fig1 = figure(1); obj1 = datacursormode(fig1); set(obj1,'Updatefcn',@cursorprecision); clf;

ax(1) = subplot(2,1,1); hold on;
plot(Power_Hip(:,2),Power_Hip(:,1),'r')
plot(Power_Inn(:,2),Power_Inn(:,1),'b')
plot(Power_Out(:,2),Power_Out(:,1),'g')
plot(Power_Steer(:,2),Power_Steer(:,1),'k')
legend('Hip', 'Inner Leg', 'Outer Leg', 'Steering')
title('Motor Power')
xlabel('Time (ms)')
ylabel('Power (W)')

ax(2) = subplot(2,1,2); hold on;
plot(HS(:,2),HS(:,1),'b:')
plot(RC(:,2),RC(:,1),'r:')
% % plot(HS_Inn_R(:,2),HS_Inn_R(:,1),'b-')
% % plot(HS_Out_L(:,2),HS_Out_L(:,1),'g:')
% % plot(HS_Out_R(:,2),HS_Out_R(:,1),'g-')
legend('Heel Strikes', 'RC control used')
xlabel('Time (ms)')
ylabel('Value')
linkaxes(ax,'x'); %%%%%%%%%%%%%%%%%%%%%%%%%%% NOOPI ADDED THIS 1st MARCH 2013

[X Y] = ginput(2);  %Enter the START and END time of the walk on the graph
Start = X(1);
Finish = X(2);
Time = Finish-Start;

% Truncate the plots to show the motor use

Idx_Hip = Power_Hip(:,2) >= Start & Power_Hip(:,2) <= Finish;
P_Hip = Power_Hip(Idx_Hip,:);
P_Hip_Avg = trapz(P_Hip(:,2),P_Hip(:,1))/Time;

Idx_Inn = Power_Inn(:,2) >= Start & Power_Inn(:,2) <= Finish;
P_Inn = Power_Inn(Idx_Inn,:);
P_Inn_Avg = trapz(P_Inn(:,2),P_Inn(:,1))/Time;

Idx_Out = Power_Out(:,2) >= Start & Power_Out(:,2) <= Finish;
P_Out = Power_Out(Idx_Out,:);
P_Out_Avg = trapz(P_Out(:,2),P_Out(:,1))/Time;

Idx_Steer = Power_Steer(:,2) >= Start & Power_Steer(:,2) <= Finish;
P_Steer = Power_Steer(Idx_Steer,:);
P_Steer_Avg = trapz(P_Steer(:,2),P_Steer(:,1))/Time;

Idx_Dist = Dist_Tot(:,2) >= Start & Dist_Tot(:,2) <= Finish;
Dist_Tot = Dist_Tot(Idx_Dist,:);
Total_Distance_Travelled = Dist_Tot(end,1) - Dist_Tot(1,1);

Idx_MCU_Volt = MCU_Volt(:,2) >= Start & MCU_Volt(:,2) <= Finish;
MCU_Volt = MCU_Volt(Idx_MCU_Volt,:);

Idx_MCU_Curr = MCU_Curr(:,2) >= Start & MCU_Curr(:,2) <= Finish;
MCU_Curr = MCU_Curr(Idx_MCU_Curr,:);

% % MCU_Power = MCU_Curr.*Idx_MCU_Volt;   %Total power
% % MCU_Power_Avg = trapz(MCU_Power(:,2),MCU_Power(:,1))/Time;

%Plot Power
fig2 = figure(2); obj2 = datacursormode(fig2); set(obj2,'Updatefcn',@cursorprecision); clf;
hold on;
plot(P_Hip(:,2)*MS_to_S,P_Hip(:,1),'r')
plot(P_Inn(:,2)*MS_to_S,P_Inn(:,1),'b')
plot(P_Out(:,2)*MS_to_S,P_Out(:,1),'g')
plot(P_Steer(:,2)*MS_to_S,P_Steer(:,1),'k')
legend('Hip', 'Inner Leg', 'Outer Leg', 'Steering')
title('Motor Power')
xlabel('Time (s)')
ylabel('Power (W)')

%Add up total work:
% % disp(' ')
% % disp(['Total Power Estimate:      ' num2str(MCU_Power_Avg) ' (W)']);
disp(' ')
disp(['Avg. Hip Power:      ' num2str(P_Hip_Avg) ' (W)']);
disp(['Avg. Inn Power:      ' num2str(P_Inn_Avg) ' (W)']);
disp(['Avg. Out Power:      ' num2str(P_Out_Avg) ' (W)']);
disp(['Avg. Steering Power: ' num2str(P_Steer_Avg) ' (W)']);
disp(['Total Time:          ' num2str(Time*MS_to_S) ' (s)']);
disp(['Estimated Distance:  ' num2str(Total_Distance_Travelled) ' (m)']);

% % % Compute Speed
% % Speed = Distance_Traveled / (Time*MS_to_S);
% % 
% % % Compute COT
% % COT = Average_Power/(Weight*Speed);

%Display COT results
% % disp(['Speed: ' num2str(Speed) '(m/s)'])
% % disp(['Weight: ' num2str(Weight) '(N)'])
% % disp(' ')
% % disp(['Cost of Transport: ' num2str(COT)])
