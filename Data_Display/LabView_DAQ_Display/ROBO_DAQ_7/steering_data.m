B = textread('Test-20121031-024856.txt'); % just turned left to the max. 

figure
hold on; 
ID= 116; plot(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2)/100 ,'-r.', 'displayname' , 'com curr')
% ID= 117; plot(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2),'-r.', 'displayname' , 'stiffness')
% ID= 118; plot(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2),'-b.', 'displayname' , 'dampness')
% ID= 112; plot(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2),'-g.', 'displayname' , 'steer angle')
ID= 110; plot(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2),'-g.', 'displayname' , 'mot curr')
%  ID= 111; plot(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2),'-b.', 'displayname' , 'mot target curr')
% ID= 114; plot(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2),'-b.', 'displayname' , 'shutdown')
% ID= 382; plot(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2),'-c.', 'displayname' , 'null-s-dang')
% ID= 149; plot(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2),'-m.', 'displayname' , 'rc-0')
% ID= 150; plot(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2),'-b.', 'displayname' , 'rc-1')
% % ID= 178; plot(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2),'-k.', 'displayname' , 'steer fsm state')
legend show
