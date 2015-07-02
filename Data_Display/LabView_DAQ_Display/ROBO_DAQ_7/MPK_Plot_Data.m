close all
clear

N_Select = 5;   %Select the N_Select most frequently sampled data channels

% SELECT THE LATEST FILE
File_List = dir('Test*.txt');
fileName = File_List(end).name;
%  
% read data 
Raw_Data = textread(fileName); % B = [can_id , value , time] from the chosen file

%Figure out what data is being plotted:
All_ID = sort(unique(Raw_Data(:,1)));
Freq = histc(Raw_Data(:,1),All_ID);
plot(All_ID,Freq,'k.','MarkerSize',15)

%Sort the Data IDs by frequency
ref_id = 1:length(Freq);
sorted_id = sortrows([Freq(:), ref_id(:)],-1);
All_ID = All_ID(sorted_id(:,2));

%List of IDs that correspond to the most frequently sampled channels
Data_ID = All_ID(1:N_Select);

figure(1); clf; hold on;
colors = ['r-','g-.','b:','k--'];
for i=1:N_Select
    Idx = Raw_Data(:,1) == Data_ID(i);
    Time = Raw_Data(Idx,3)/1000;
    Data = Raw_Data(Idx,2);
    C_idx = mod(i-1,length(colors))+1;
    c = colors(C_idx);
    plot(Time,Data,c)
end
title('Data collected from Ranger')
xlabel('Time (seconds)')
ylabel('Value (mixed units)')

legend(...
    ['CAN ID ' num2str(Data_ID(1))],...
    ['CAN ID ' num2str(Data_ID(2))],...
    ['CAN ID ' num2str(Data_ID(3))],...
    ['CAN ID ' num2str(Data_ID(4))],...
    ['CAN ID ' num2str(Data_ID(5))]);


