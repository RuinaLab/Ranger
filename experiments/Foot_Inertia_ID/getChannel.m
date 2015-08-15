function channel = getChannel(data,ID,name)
% channel = getChannel(data,ID,name)
%
% Extracts a single data channel from the raw data produced by Rangers DAQ 
% system. It also removes duplicates and stores time stamp and data value
% into different fields of the channel struct
%
% INPUTS:
%   data = [N, 3] = matrix of raw data
%       (:,1) = ID
%       (:,2) = value
%       (:,3) = time stamp (ms)
%
%   ID = integer corresponding to the channel that should be read
%   name = human-readable name for this channel
%
% OUTPUTS:
%   channel = struct with fields:
%       .time = [1, M] = time stamps, in seconds, since data logging started
%       .data = [1, M] = data value for each time stampe
%       .id = integer channel id number
%       .name = string name for this channel
%

% Extract this channel
chan = data(data(:,1)==ID, [3,2]);

% Remove duplicate enteries:
idxRm = diff(chan(:,1))==0; 
chan = chan(~idxRm,:);

% Adjust time to be more convienant:
startTime = min(data(:,3));
time = chan(:,1)' - startTime;
time = time/1000;  %Convert to seconds (from ms)

% Create channel struct:
channel.name = name;
channel.id = ID;
channel.time = time;
channel.data = chan(:,2)';

end