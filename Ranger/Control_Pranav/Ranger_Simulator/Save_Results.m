function Save_Results(Results) %#ok<INUSD>

date_vec = fix(clock);   %Round the current date and time to the nearest second
dateTime = [];

for i=1:5
    dateTime = [dateTime num2str(date_vec(i)) '_']; %#ok<AGROW>
end
dateTime = [dateTime num2str(date_vec(6))];

directory = 'SNOPT_Data_Runs\';
extension = '.mat';
name = 'DATA_';

if Results.SnoptOutput.inform ~= 1
    %Then SNOPT returned a failure code
    FailCode = ['_FAILED_' num2str(Results.SnoptOutput.inform)];  
else
    %SNOPT finished successfully
    FailCode = '';
end

SaveName = [...
    directory,...    
    name,...
    dateTime,...
    FailCode,...
    extension];

save(SaveName,'Results')

end
    