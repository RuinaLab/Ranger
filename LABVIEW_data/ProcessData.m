% Data format:
% ID    '\t'   VALUE   '\t'    TIME  
Data = dlmread('Test7_1step.txt','\t');

th0_id = 235;  
q0_id = 193;
q1_id = 194;
qh_id = 14;

idx_th0 = Data(:,1) == th0_id;  %Get indicies for gyro date
idx_q0 = Data(:,1) == q0_id; 
idx_q1 = Data(:,1) == q1_id; 
idx_qh = Data(:,1) == qh_id; 

t_th0 = Data(idx_th0,3);
th0 = Data(idx_th0,2);

t_q0 = Data(idx_q0, 3);
q0 = Data(idx_q0, 2);

t_q1 = Data(idx_q1, 3);
q1 = Data(idx_q1, 2);

t_qh = Data(idx_qh, 3);
qh = Data(idx_qh, 2);

%hold on;
figure(1); clf;
plot(t_th0,th0,'r', t_q0,q0,'b', t_q1,q1,'y', t_qh,qh,'g');
legend('th0', 'q0', 'q1', 'qh');

%fprintf('Mean gyro rate: %8.8f \n',mean(gyro));
