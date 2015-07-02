w = 150;        % width of the graph, ms

% file = 'Test-20130228-190033.txt'; % straight
% file = 'Test-20130228-190400.txt'; % 0.1 max turn
% file = 'Test-20130228-190622.txt'; % 0.07 max turn

 file = 'Test-20130301-114916.txt'; tits= 'walking straight % swh threshholds 0.01';
%file = 'Test-20130301-115244.txt'; tits=  'walking with slight turn (target steering angle 0.07) % swh threshhols 0.01';
 %file = 'Test-20130301-115627.txt';  tits= 'walking with hard turn (target steering angle 0.1) % swh threshholds 0.01';



% load the walk data
B = textread(file);

id_h = 222;     % index of the ankle-to-ankle height can_id
id_esl = 215;   % index of the ID_E_SWING_LEG can_id

h = B(B(:,1)==id_h,2);      % height data
esl = B(B(:,1)==id_esl,2);      % ID_E_SWING_LEG data
t_h = B(B(:,1)==id_h,3);    % time data of the height
t_esl = B(B(:,1)==id_esl,3);    % time data of the ID_E_SWING_LEG data

% choose time interval of interest
t0 = 320000;%0;
t1 = 360000;%t_esl(end);
ind = (t_h>=t0) & (t_h<t1);
t_h = t_h(ind);
h = h(ind);
ind = (t_esl>=t0) & (t_esl<t1);
t_esl = t_esl(ind);
esl = esl(ind);

% find instants of heelstrike
ind_hs_inner = find(esl(2:end)-esl(1:end-1) < 0) + 1;
t_hs_inner = t_esl(ind_hs_inner);
ind_hs_outer = find(esl(2:end)-esl(1:end-1) > 0) + 1;
t_hs_outer = t_esl(ind_hs_outer);


%%% inner legs heelstrike %%%

ind0 = 0 * t_hs_inner;
ind1 = ind0;
for i = 1:length(t_hs_inner)  % go through each heelstrike
    ind0(i) = find(t_h >= t_hs_inner(i)-w, 1);    % index of height data corresponding to w ms before the heelstrike
    ind1(i) = find(t_h >= t_hs_inner(i), 1);      % index of height data corresponding to the heelstrike
end;

% plot everything on the same graph
figure(1); clf;
title({'Inner legs heelstrike', tits});
grid on; hold all;
for i = 1:length(ind0)
    plot(t_h(ind0(i):ind1(i))-t_hs_inner(i), h(ind0(i):ind1(i)));
end;


%%% outer legs heelstrike %%%

ind0 = 0 * t_hs_outer;
ind1 = ind0;
for i = 1:length(t_hs_outer)  % go through each heelstrike
    ind0(i) = find(t_h >= t_hs_outer(i)-w, 1);    % index of height data corresponding to w ms before the heelstrike
    ind1(i) = find(t_h >= t_hs_outer(i), 1);      % index of height data corresponding to the heelstrike
end;

% plot everything on the same graph
figure(2); clf;
title({'Outer legs heelstrike',tits});
grid on; hold all;
for i = 1:length(ind0)
    plot(t_h(ind0(i):ind1(i))-t_hs_outer(i), h(ind0(i):ind1(i)));
end;


figure(3)
hold on; 
ID= 175; stairs(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2)/100 ,'-r.', 'displayname' , 'hip fsm state')
ID= 176; stairs(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2)/100,'-g.', 'displayname' , 'inner foot fsm')
ID= 177; stairs(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2)/100 ,'-b.', 'displayname' , 'outer foot fsm')
ID= 215; stairs(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2)/100,'-c.', 'displayname' , 'id e swing leg')
ID= 222; plot(B((B(:,1)==ID), 3),B((B(:,1)==ID), 2),'-m.', 'displayname' , 'ank to ank height')
title({'stuff', tits});
grid on;
legend show; 