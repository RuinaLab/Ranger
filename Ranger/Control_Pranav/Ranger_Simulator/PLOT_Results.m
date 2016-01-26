function PLOT_Results(Results,N)

PlotResults = FormatResults(Results)

%N is the figure to start plotting on
if nargin == 1
    N = 1;
end


%% Plot the actuator command currents:
figure(N+1); clf;
    subplot(2,2,1)
    %BLANK%
    axis off
    subplot(2,2,3)
    title('Total Mechanical Energy')
        PlotNames={};
        PlotNames{1} = 'Energy';
        MakePlot(PlotNames,PlotResults)
        ylabel('Mechanical Energy (J)')
    
    subplot(2,2,2)
    title('Hip Angle')
        PlotNames={};
        PlotNames{1} = 'HipAngle';
        MakePlot(PlotNames,PlotResults)
        ylabel('Angle (rad)')
    subplot(2,2,4)
    title('Hip Rate')
        PlotNames={};
        PlotNames{1} = 'HipRate';
        MakePlot(PlotNames,PlotResults)
        ylabel('Angle (rad/s)')
        
%% Plot all of the joint angles
figure(N+2); clf;

    subplot(2,2,1)
        PlotNames={};
        PlotNames{1} = 'FootJointAngle';
        PlotNames{2} = 'FootMotorAngle';
        MakePlot(PlotNames,PlotResults)
        ylabel('Angle (rad)')

    subplot(2,2,3)
        PlotNames={};
        PlotNames{1} = 'FootJointRate';
        PlotNames{2} = 'FootMotorRate';
        MakePlot(PlotNames,PlotResults)
        ylabel('Angle (rad/s)')
        
        
    subplot(2,2,2)
        PlotNames={};
        PlotNames{1} = 'FootAbsAngle';
        MakePlot(PlotNames,PlotResults)
        ylabel('Angle (rad)')

    subplot(2,2,4)
        PlotNames={};
        PlotNames{1} = 'FootAbsRate';
        MakePlot(PlotNames,PlotResults)
        ylabel('Angle (rad/s)')
        
        
%% Plot the hip coordinates:
figure(N+3); clf;

    subplot(2,3,1)
        PlotNames={};
        PlotNames{1} = 'HipPosX';
        MakePlot(PlotNames,PlotResults)
        ylabel('Horizontal Position (m)')

    subplot(2,3,4)
        PlotNames={};
        PlotNames{1} = 'HipVelX';
        MakePlot(PlotNames,PlotResults)
        ylabel('Horizontal Speed (m/s)')
        
    subplot(2,3,2)
        PlotNames={};
        PlotNames{1} = 'HipPosY';
        MakePlot(PlotNames,PlotResults)
        ylabel('Vertical Position (m)')

    subplot(2,3,5)
        PlotNames={};
        PlotNames{1} = 'HipVelY';
        MakePlot(PlotNames,PlotResults)
        ylabel('Vertical Speed (m/s)')
        
    subplot(2,3,[3 6])
        x = PlotResults.HipPosX;
        y = PlotResults.HipPosY;
        plot(x,y)
        title('Trace for the position of the hip')
        xlabel('Horizontal Position (m)')
        ylabel('Vertical Position (m)')
        axis equal
        
%% Plot motor commands and reaction forces
figure(N+4); clf;
    subplot(2,1,1)
    title('Actuator Command Currents')
        PlotNames={};
        PlotNames{1} = 'HipMotorCurrent';
        PlotNames{2} = 'AnkleCurrent';
        MakePlot(PlotNames,PlotResults)
        ylabel('Command Current (A)')
        
   subplot(2,1,2)
   title('Ground Reaction Forces')
       PlotNames = {};
       PlotNames{1} = 'FootVertRxn';
       MakePlot(PlotNames,PlotResults)
       ylabel('Vertical Reaction Force (N)')
        
end


function MakePlot(PlotNames,K)

    N = length(PlotNames);
    Time = K.Time;
    M = length(Time);
    
    Data = zeros(N,M);
    
    hold on;
    for i=1:N
       Data(i,:) = K.(PlotNames{i}); 
    end
    plot(Time,Data);   
    legend(PlotNames,'Location','NorthWest')
    xlabel('Time (s)');
    
    %Plot when heel strike occurs:
    Extents = axis;
    for i=1:length(K.SwitchTime)
        T_HS = K.SwitchTime(i);
        plot(T_HS*[1,1],Extents(3:4),'k:')  %Plot a vertical dotted line at heel strike
    end
    hold off
    
end


function K = ConcatPlot(R)

%This function concatentates the data so that a single foot can show all of
%the information. It goes [Stance, Swing];
	
    K.Time = [R.Time, (R.Time+R.Time(end)+eps)];   %Time for the whole thing
    K.SwitchIdx = [1,R.SwitchIdx, length(R.Time)+1,length(R.Time)+R.SwitchIdx];   %Index where the feet switch of heel strike happens
    K.SwitchTime = K.Time(K.SwitchIdx);
    
    K.FootAbsAngle = [R.StanceFootAbsAngle, zeros(size(R.StanceFootAbsAngle))];   %HACK - assume swing foot stays level
    K.FootAbsRate = [R.StanceFootAbsRate, zeros(size(R.StanceFootAbsRate))];   %HACK - assume swing foot stays level
    
    K.FootJointAngle = [R.StanceFootJointAngle, R.SwingFootJointAngle];
    K.FootJointRate = [R.StanceFootJointRate, R.SwingFootJointRate];
    K.FootMotorAngle = [R.StanceFootMotorAngle, R.SwingFootMotorAngle];
    K.FootMotorRate = [R.StanceFootMotorRate, R.SwingFootMotorRate];
    
    K.HipAngle = [R.HipAngle, -R.HipAngle];
    K.HipRate = [R.HipRate, -R.HipRate];
    
    Diff_X_Pos = R.HipPosX(end) - R.HipPosX(1);   %How far did things move forward?
    K.HipPosX = [R.HipPosX, (R.HipPosX+Diff_X_Pos)];
    K.HipPosY = [R.HipPosY, R.HipPosY];
    K.HipVelX = [R.HipVelX, R.HipVelX];
    K.HipVelY = [R.HipVelY, R.HipVelY];
    
    K.Energy = [R.Energy, (R.Energy+R.Energy(end))];
    
    K.HipMotorCurrent = [R.HipMotorCurrent, -R.HipMotorCurrent];
    K.AnkleCurrent = [R.AnkleCurrent1, R.AnkleCurrent2];
    
    K.FootVertRxn = [R.StanceFootVertRxn, R.SwingFootVertRxn];
    
end