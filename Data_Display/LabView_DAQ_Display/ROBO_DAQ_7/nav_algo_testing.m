walkdata = textread('Test-20130227-194916.txt');

id=393; t = walkdata(walkdata(:,1)==id,3);
id=394; dist = walkdata(walkdata(:,1)==id,2);
id=390; t = walkdata(walkdata(:,1)==id,3);
id=390; dist = walkdata(walkdata(:,1)==id,2);
ind = find(t(1:end-1)-t(2:end));
t = t(ind);
dist = dist(ind);
id=391; dist_old = walkdata(walkdata(:,1)==id,2);
id=391; t_do = walkdata(walkdata(:,1)==id,3);
ind = find(t_do(1:end-1)-t_do(2:end));
t_do = t_do(ind);
% ind = find(t_do-t);
dist_old = dist_old(ind);
ind = [];
for i = 1:length(t)
    k = find(t_do-t(i), 1);
    ind = [ind, k];
end;

step=0.67;
max_ang = 0.1;
min_r = step/max_ang;
D=1.57;
theta = (dist - dist_old)/step;
A = 1/step;
B = (D - max_ang) ./ (min_r - step*max_ang);
C = (B*step + 1.0)*max_ang;

stairs(t, A*abs(dist)); grid on; hold all;
stairs(t, B*abs(dist)+C, 'red');

id=385; stairs((walkdata(walkdata(:,1)==id,3)), (walkdata(walkdata(:,1)==id,2)), 'green', 'displayname', 'RC is used', 'LineWidth',2) ;
