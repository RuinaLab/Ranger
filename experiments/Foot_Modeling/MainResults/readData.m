function data = readData(fileName,channelList)

data = dlmread(fileName,'\t');

% Only keep desired channels
idxKeep = false(size(data(:,1)));
for i=1:length(channelList)
    idxKeep = idxKeep | channelList(i)==data(:,1); 
end

% Remove data points at zero time:
idxKeep = idxKeep & data(:,3)~=0;

data = data(idxKeep,:);

end