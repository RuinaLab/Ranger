function plotData(Data, tSpan)

if nargin == 1 
    idx = true(size(Data.time));
elseif isempty(tSpan)
    idx = true(size(Data.time));
else
    idx = Data.time > tSpan(1)  & Data.time < tSpan(2);
end

plot(Data.time(idx), Data.data(idx));

end