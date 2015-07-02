close all
clear
%  % READ DATA FILE
%  [fileName, filePath] = uigetfile({'*.txt'}, 'Please select the data file.');
%  if(exist(fileName , 'file')==0) curr_dir = pwd; copyfile([filePath fileName] , curr_dir); end; % copies the selected file to current dir if it isn't there.
 
% SELECT THE LATEST FILE !!!!
 y = dir('Test*.txt');
fileName = y(end).name;
%  
% read data 
% B = textread(fileName); % B = [can_id , value , time] from the chosen file
B = textread('Test-20130628-173658.txt');
%plot
fig = figure(1); n=1; obj = datacursormode(fig); set(obj,'Updatefcn',@cursorprecision);
hold on;

% id = 37; stairs((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), 'green', 'displayname', 'mcfo right ankle angle', 'LineWidth',2);
% id = 65; stairs((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), 'black', 'displayname', 'mcfi mid ankelang', 'LineStyle','--');
% 
% id = 49; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), 'blue', 'displayname', 'mcfo left heel sense', 'LineWidth',2)  ;
% id = 50; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), 'cyan', 'displayname', 'mcfo right heel sense') ;
% id = 77; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), 'red', 'displayname', 'mcfi left heel sense', 'LineWidth',2) ;
% id = 78; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), 'magenta', 'displayname', 'mcfi right heel sense') ;
% 
% id = 112; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), 'blue', 'displayname', 'steer angle', 'LineWidth',2);
% id = 116; stairs((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), 'red', 'displayname', 'commanded angle', 'LineWidth',2);

% id = 385; stairs((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), 'green', 'displayname', 'RC is used', 'LineWidth',2) ;
% id = 388; stairs((B(B(:,1)==id,3)), (B(B(:,1)==id,2))/5, 'black', 'displayname', 'camera signal', 'LineWidth',2) ;

id = 65; stairs((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), 'blue', 'displayname', 'ankle angle', 'LineWidth',2);



% ax(1) = subplot(n,1,1);hold on 
% id = 145; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), '-m.', 'displayname', 'command current');
 
%   id = 14; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), '-k', 'displayname', 'hip angle', 'linewidth', 2);
 % id = 215; plot((B(B(:,1)==id,3)), 0*(B(B(:,1)==id,2)), '-co', 'displayname', 'ID E SWING LEG', 'linewidth', 2);
 
%   id = 152; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), '-r', 'displayname', 'camera signal')% 
%   id = 110; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), '-b', 'displayname', 'motor current', 'linewidth', 2)% 
% 
% id = 11 ; plot((B(B(:,1)==id,3)), [(B(B(:,1)==id,2))], '-b', 'displayname', 'actual hip current')
% id = 18 ; plot((B(B(:,1)==id,3)), [(B(B(:,1)==id,2))], ':b', 'displayname', 'command hip curr', 'linewidth', 2)

legend show
grid on
% linkaxes(ax,'x') % for subplots
% 
% fig = figure(2); n=1; obj = datacursormode(fig); set(obj,'Updatefcn',@cursorprecision);
% hold on;
% % ax(1) = subplot(n,1,1);hold on 
% id = 11 ; plot((B(B(:,1)==id,3)), [(B(B(:,1)==id,2))], '-b.', 'displayname', 'actual hip current')
% id = 18 ; plot((B(B(:,1)==id,3)), [(B(B(:,1)==id,2))], ':b.', 'displayname', 'command hip curr', 'linewidth', 2)
% 
% id = 145 ; plot((B(B(:,1)==id,3)), -[(B(B(:,1)==id,2))]-.0043, '-c', 'displayname', 'stance rate (gyro)')
% id = 229 ; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2))  , ':c', 'displayname', 'est stance rate', 'linewidth', 2)
% 
% id = 21; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), '-r', 'displayname', 'filtered hip rate')% 
% id = 230; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), ':r', 'displayname', 'est hip rate', 'linewidth', 2)% 
% 
% legend show
% grid on
% % linkaxes(ax,'x') % for subplots
% 
% 
% 
% %% swing foot height code
% 
% % fig = figure(); n=2; obj = datacursormode(fig); set(obj,'Updatefcn',@cursorprecision);
% % hold on;
% % id = 193 ; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), '-r', 'displayname', 'swing_foot_height')% 
% % id = 219 ; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), '-k', 'displayname', 'ankle_ankle_height')% 
% % id = 204 ; plot((B(B(:,1)==id,3)), 0*(B(B(:,1)==id,2)), 'r.', 'displayname', 'distance')% 
% % id = 214 ; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), '-', 'displayname', 'prepush to hs time')% 
% % 
% % id = 49 ; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), '--b', 'displayname', 'outer left')% 
% % id = 50 ; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), '-b', 'displayname', 'outer right')% 
% % id = 77 ; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), '--c', 'displayname', 'inner left')% 
% % id = 78 ; plot((B(B(:,1)==id,3)), (B(B(:,1)==id,2)), '-c', 'displayname', 'inner right')% 
% 
% %%
% 
% 
