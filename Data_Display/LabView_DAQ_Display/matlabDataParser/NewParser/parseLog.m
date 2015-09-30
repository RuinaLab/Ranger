function allData = parseLog(path)
% Given relative file path (a string), return a big struct containing
% smaller structs -- one for each data field -- containing timestamp and
% value data.
% e.g. '../ROBO_DAQ_7/Test-20150927-163251.txt'

%Notes: Duplicate data points. TODO?

%Get the file ID for the input file
fid = fopen(path);

%Scan through assuming 3 columns for everything. Assume the first column is
%a string to start with (due to the header file to ID definitions).
rawScan = textscan(fid,'%s%f%d');

%Convert all in first column to numbers. 
firstCol = str2double(rawScan{1});

%NaN values will be the ones which actually were supposed to be strings.
firstNumIndex = sum(isnan(firstCol))+1;

% These are the labels with their associated IDs
varNames = rawScan{1}(1:firstNumIndex-1);
varIDNums = rawScan{2}(1:firstNumIndex - 1);

%Make a struct of structs. Top level has fields with the names of the
%variables from labview. Lower level has Name, ID#, timestamps, and values
%as fields.
for i = 1:length(varNames)
    allData.(varNames{i}) = struct('name',varNames{i},'ID',varIDNums(i),'timestamps',[],'values',[]); 
end


%These are the unsorted logged data. Order is: [ID, value, time (MS)]
dataIDNums = firstCol(firstNumIndex:end); %want the version with numbers, not strings for this one.
dataValues = rawScan{2}(firstNumIndex:end);
dataTimestamp = rawScan{3}(firstNumIndex:end);


%Sorting here:
%  IDs are on columns, actual data points are on rows.
%  When there's a one, data point n is of category m (nxm matrix output).
matchPoints = repmat(dataIDNums,[1,length(varIDNums)]) == repmat(varIDNums',[length(dataIDNums),1]);


for i=1:1:length(varIDNums)
    potentialTime = dataTimestamp(matchPoints(:,i));
    potentialData = dataValues(matchPoints(:,i));
    
    allData.(varNames{i}).timestamps = potentialTime(potentialTime ~= 0);
    allData.(varNames{i}).values = potentialData(potentialTime ~= 0); %Only keep the nonzero timestamp ones.

end
