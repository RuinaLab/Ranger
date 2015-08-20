% Data format:
% ID    '\t'   VALUE   '\t'    TIME  
Data = dlmread('Test1.txt','\t');

qr_int_id = 194;
qr_geo_id = 246;

idx_qr_int = Data(:,1) == qr_int_id;  %Get indicies for gyro date
idx_qr_geo = Data(:,1) == qr_geo_id; 

t_qr_int = Data(idx_qr_int, 3);
qr_int = Data(idx_qr_int, 2);

t_qr_geo = Data(idx_qr_geo, 3);
qr_geo = Data(idx_qr_geo, 2);

%hold on;
figure(1); clf;
plot(t_qr_int,qr_int,'r', t_qr_geo,qr_geo,'bo');
legend('qr_int', 'qr_geo');

%fprintf('Mean gyro rate: %8.8f \n',mean(gyro));
