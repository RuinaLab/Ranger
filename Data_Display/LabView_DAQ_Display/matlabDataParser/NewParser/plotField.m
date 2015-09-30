function figHandle = plotField( dataStruct, fieldName, typeFlag )
%Plotter function -- Give it the full data structure from parseLog and a
%field name (CAN BE A CELL ARRAY -- will make subplots).

figHandle = figure;

switch typeFlag
    case 'subplots'
        subWidth = ceil(sqrt(length(fieldName))); %Try to make roughly squarish subplot arrangements.
        subLength = ceil(length(fieldName)/subWidth);
        for i = 1:length(fieldName)
            subplot(subWidth,subLength,i); %figure out the position of the next subplot.
            plot(dataStruct.(fieldName{i}).timestamps,dataStruct.(fieldName{i}).values);
            xlabel('Time (ms)');
            ylabel(fieldName{i});
            
        end
    case 'overlay'
        hold on
        
        X = [];
        Y = [];
        for i = 1:length(fieldName)
            X(:,end+1) = dataStruct.(fieldName{i}).timestamps;
            Y(:,end+1) = dataStruct.(fieldName{i}).values;
        end
        plot(X,Y)
        xlabel('Time (ms)')
        legend(fieldName);
    otherwise
        error('Invalid typeFlag for plotting');
        
end
    
end