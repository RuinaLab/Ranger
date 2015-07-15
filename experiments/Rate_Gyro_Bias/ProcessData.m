% Data format:
% ID    '\t'   VALUE   '\t'    TIME  
Data = dlmread('dataFile1.txt','\t');

gyro_id = 269;  

idx_gyro = Data(:,1) == gyro_id;  %Get indicies for gyro date

time = Data(idx_gyro,3);
gyro = Data(idx_gyro,2);

plot(time,gyro);

fprintf('Mean gyro rate: %8.8f \n',mean(gyro));
