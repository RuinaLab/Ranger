clear
clc
close all
% read data
%% tests in the lab (4th march) (7 and 10mm)
%%%% start_index = index of distance data when the first step was taken, =2 if distance(1)=0,  =1 if distance(1)= first step length .
% B = textread('Test-20110304-194416.txt'); actualdist = 16; start_index=1;  fudge=0; col = 'r'; % outside and away % )
B = textread('Test-20110306-050639.txt'); actualdist = 16; start_index=2; fudge=0; col = 'r'; % outside and towards% 

% B = [can_id , value , time] from the chosen file
%% PARAMETERS (same as in ranger)
L_leg = 0.960; %%leg length (hip joint to ankle joint)
foot_radius_inner  = 0.23; %% foot radius 20 %% this is dubious, its hard to measure its from 15-22cm, 
foot_radius_outer  = 0.168; %% foot radius 20 %% this is odubious, its hard to measure its from 15-22cm, 
d_ankle2sweetspot = .054; %% this distance I trust within +- 0.0005 meters 
d_ac_inner = 0.23-.054; 
d_ac_outer = 0.168-.054; %%d_ac=ankle length = distance from ankle_joint to foot_circle's center. %%its being written as FootRadius-d_ankle2sweetspot,  %%I TRUST the ankle2sweetspot (0.054 m) MORE
ankle_angle_offset = 1.68;%%1.71;%% 96.2 deg  %% offset = measured from can id - real_ankle_angle (between leg and the ankle-2-foot-centre line, foot being thought as a big circle). add pi to this real_ankle_angle and you get the angle to sweet spot 
foot_absang_calibration_constant = 1.75;
robot_mass = 9.84; %kilograms
int_reset_factor = .1; 
overheads = 5.25; % robot overhead cost.

%% in all of the above the high speed canid's are
% absang LI LO FI FO: 188 189 191 192 
% LO integration reset amount: 208
% foot contact sensor outer, inner: 50 78 
% joint angles: hip, mid-ankle, outer-left, outer-right: 14 65 91 37  
% sfh, ankle-ankleheight 193 185 
% total distance 204  
% imu-rate: 145

% 11 14 15 21  33 35 37 40 41  44 50  62 63 65 68 69 70 78   99 145 175 176   177 183 184 188     189 190 191  192    193 204 206  207     195 196 197 198
%      
%% can id names
hipcurrent = 11; focurrent =35; ficurrent = 62; ... currents
li= 188;  lo=189;  fi=191;  fo=192;  int_lo_reset=207; ... abs angs, reset
fisense=78; fosense=50; ... contact sense
hip=14; iank=65; orank=37; olank=99; ... joint angle
fo_motor_ang = 33 ;fi_motor_ang = 63 ;... motor angles
hrate=21; ehrate=190; imurate=145; elo_rate= 184; eli_rate = 183; ...  ang rates
swtsptheit = 193; ... sfh
hip_fsm = 175; fi_fsm=176; fo_fsm=177; ... fsm
distance=204;  mb_est_time = 206;  ...  distance time
hip_power=15; fi_power=70; fo_power=44;   ... powers
midst_legr =195; midst_hipr=196; midst_hip=197; midst_time=198; ...  mistancestuff 
folhs=41 ; forhs = 40; filhs = 69 ; firhs = 68; ... heel strike ids

canlist= {'hipcurrent'; 'focurrent'; 'ficurrent'; 'li';  'lo';  'fi';  'fo';  'int_lo_reset'; ...
'fisense'; 'fosense'; 'hip'; 'iank'; 'orank'; 'olank'; ...
'hrate'; 'ehrate'; 'imurate'; 'elo_rate'; 'eli_rate'; 'swtsptheit'; 'fo_motor_ang'; 'fi_motor_ang' ; ...
'hip_fsm'; 'fi_fsm'; 'fo_fsm'; 'distance';  'mb_est_time';  ...
'hip_power'; 'fi_power'; 'fo_power'; 'midst_legr'; 'midst_hipr'; 'midst_hip'; 'midst_time'};

%% extract data
for i=1:length(canlist)
    eval([canlist{i} 'data=(B(B(:,1)==' 'eval(canlist{i})' ',2));']); eval([canlist{i} 'time=(B(B(:,1)==' 'eval(canlist{i})' ',3));']); % defines a variable like: LItime=(B(B(:,1)==id,3));
end
 
%% removed repeated time stamps (will help in interpolation)
for i=1:length(canlist)
    eval([canlist{i} 'data(diff(' canlist{i} 'time)==0)=[];']);  eval([canlist{i} 'time(diff(' canlist{i} 'time)==0)=[];']); %  LIdata(diff(LItime)==0)=[]; LItimediff(LItime)==0)=[];
end

%% clip data after some time
% (because it tends to come back to zero time stamp)
t_end = max(distancetime)+10 ; % 10 ms after the last time distance was updated (ie last heel strike)
for i=1:length(canlist)
    index = eval(['find(' canlist{i} 'time>=t_end,1);']); %returns the index of time when it goes above t_end
    if(isempty(index)); index = eval(['find(' canlist{i} 'time==max(' canlist{i} 'time));']); end
    eval([canlist{i} 'data=' canlist{i} 'data(1:index);' ]);  eval([canlist{i} 'time=' canlist{i} 'time(1:index);' ]);
end


%% interpolate the required data
%t_start = min(hiptime)+10; % starting time
t_start = distancetime(start_index) -25; % starting time 
if(length(int_lo_resettime)== length(distancetime))
    display('length of imureset and distance are same, GOOD, noopi check if they are same');
else
    display('length of imureset and distance are not same, BAD, check the lo once reset data. ');
end

if(distancetime(start_index)==0)
    display('distance time calculation is NOT ok')
elseif (distancetime(start_index)>.05 && distancetime(start_index)< .45)
    display('distance time calculation is OK')
end

time_interval= t_start:1:t_end;

bad_index = find(diff(fosensetime)<0)+1 ; fosensedata(bad_index)=[]; fosensetime(bad_index)=[];    %remove indices in fosensedata where timestamp go back for once
bad_index = find(diff(olanktime)<0)+1 ;     olankdata(bad_index)=[];   olanktime(bad_index)=[];    %remove indices in fosensedata where timestamp go back for once
bad_index = find(diff(oranktime)<0)+1 ;     orankdata(bad_index)=[];   oranktime(bad_index)=[];    %remove indices in fosensedata where timestamp go back for once

intp_hipdata     = interp1(hiptime, hipdata, time_interval); % interpolate data to **** 1ms sampling ********
intp_fisensedata = interp1(fisensetime, fisensedata, time_interval); 
intp_fosensedata = interp1(fosensetime, fosensedata, time_interval);
intp_iankdata  = interp1(ianktime, iankdata, time_interval);
intp_olankdata = interp1(olanktime, olankdata, time_interval);
intp_orankdata = interp1(oranktime, orankdata, time_interval);
intp_imuratedata = interp1(imuratetime, imuratedata, time_interval);


%%  noopmade canids
display(['Distance discrepency = ' num2str((actualdist-max(distancedata))/actualdist*100) '%'])
display('I am not making correction to the distance.')
%distancedata = distancedata+linspace(0,actualdist-max(distancedata), length(distancetime))';  % corrected distance

robotveldata = [0; diff(distancedata)./diff(distancetime)/1000];                              % robot velocity
hipratetime = hiptime; hipratedata = [0; diff(hipdata)./diff(hiptime)*1000] ;                 % hip rate just from diff h-angle



%% once only reset data
lo1resetdata = lodata; 
lo1resettime = lotime;

for i=(start_index+1):length(int_lo_resettime)-1
    indices = find(lotime<int_lo_resettime(i+1) & lotime>=int_lo_resettime(i) );
    lo1resetdata(indices) =lodata(indices)+int_reset_factor*sum(int_lo_resetdata((start_index+1):i)); 
end

intp_lo1resetdata = interp1(lo1resettime, lo1resetdata, time_interval);   
intp_li1resetdata = intp_lo1resetdata-intp_hipdata;
intp_fo1resetdata = intp_lo1resetdata+intp_orankdata-ankle_angle_offset;
intp_fi1resetdata = intp_li1resetdata+intp_iankdata -ankle_angle_offset;

%% data reset every step (abs angs, robot state, foot height)

intp_lodata = zeros(size(time_interval));
intp_lidata = zeros(size(time_interval));
intp_fidata = zeros(size(time_interval));
intp_fodata = zeros(size(time_interval));
robot_state = zeros(size(time_interval)); % initialize 1= inner leg swing, 0 = outer
calc_swtsptheitdata = zeros(size(time_interval));
calc_sfhdata = zeros(size(time_interval));
calc_swtsptheit1resetdata = zeros(size(time_interval));
calc_sfh1resetdata = zeros(size(time_interval));
calc_lodata = zeros(size(time_interval));

% range = find(time_interval==distancetime(2))+5:length(time_interval);
% calc_lodata(range) = intp_lodata(range(1))+cumsum(-intp_imuratedata(range))*.001;
    
for i = start_index:length(distancetime)-1
    curr_slope =  0;
        if(rem(i,2)==fudge) %% inner leg swing , outer stance
            range = find(time_interval==distancetime(i)):find(time_interval==distancetime(i+1)-1); %-1 is because I want data until 1 ms before the next heel strike point
            
            indexlo = lotime>=time_interval(range(1))& lotime<=time_interval(range(end));
            intp_lodata(range)= interp1(lotime(indexlo),lodata(indexlo) ,time_interval(range), 'linear', 'extrap');
            indexli = litime>=time_interval(range(1))& litime<=time_interval(range(end));
            intp_lidata(range)= interp1(litime(indexli),lidata(indexli) ,time_interval(range), 'linear', 'extrap');
            indexfo = fotime>=time_interval(range(1))& fotime<=time_interval(range(end));
            intp_fodata(range)= interp1(fotime(indexfo),fodata(indexfo) ,time_interval(range), 'linear', 'extrap')-foot_absang_calibration_constant;
            indexfi = fitime>=time_interval(range(1))& fitime<=time_interval(range(end));
            intp_fidata(range)= interp1(fitime(indexfi),fidata(indexfi) ,time_interval(range), 'linear', 'extrap')-foot_absang_calibration_constant;
            
            calc_lodata(range) = intp_lodata(range(1))+cumsum(-intp_imuratedata(range))*.001;
               robot_state(range)= 1+robot_state(range); %% outer leg swings
            calc_swtsptheitdata(range) = foot_radius_outer -d_ac_outer*cos(intp_fodata(range)-curr_slope)+ L_leg*(cos(intp_lodata(range)-curr_slope) -cos(intp_lidata(range)-curr_slope)) - d_ankle2sweetspot*cos(intp_fidata(range)-curr_slope);
            calc_sfhdata(range) = foot_radius_outer -d_ac_outer*cos(intp_fodata(range)-curr_slope)+ L_leg*(cos(intp_lodata(range)-curr_slope) -cos(intp_lidata(range)-curr_slope)) + d_ac_inner*cos(intp_fidata(range)-curr_slope) - foot_radius_inner ;
            calc_swtsptheit1resetdata(range) = foot_radius_outer -d_ac_outer*cos(intp_fo1resetdata(range)-curr_slope)+ L_leg*(cos(intp_lo1resetdata(range)-curr_slope) -cos(intp_li1resetdata(range)-curr_slope)) - d_ankle2sweetspot*cos(intp_fi1resetdata(range)-curr_slope);
            calc_sfh1resetdata(range) =foot_radius_outer -d_ac_outer*cos(intp_fo1resetdata(range)-curr_slope)+ L_leg*(cos(intp_lo1resetdata(range)-curr_slope) -cos(intp_li1resetdata(range)-curr_slope)) + d_ac_inner*cos(intp_fi1resetdata(range)-curr_slope) - foot_radius_inner ;       
            
        else %% outer leg swing , inner stance
            range = find(time_interval==distancetime(i)):find(time_interval==distancetime(i+1)-1); 
            
            indexlo = lotime>=time_interval(range(1))& lotime<=time_interval(range(end));
            intp_lodata(range)= interp1(lotime(indexlo),lodata(indexlo) ,time_interval(range), 'linear', 'extrap');
            indexli = litime>=time_interval(range(1))& litime<=time_interval(range(end));
            intp_lidata(range)= interp1(litime(indexli),lidata(indexli) ,time_interval(range), 'linear', 'extrap');
            indexfo = fotime>=time_interval(range(1))& fotime<=time_interval(range(end));
            intp_fodata(range)= interp1(fotime(indexfo),fodata(indexfo) ,time_interval(range), 'linear', 'extrap')-foot_absang_calibration_constant;
            indexfi = fitime>=time_interval(range(1))& fitime<=time_interval(range(end));
            intp_fidata(range)= interp1(fitime(indexfi),fidata(indexfi) ,time_interval(range), 'linear', 'extrap')-foot_absang_calibration_constant;
            
            calc_lodata(range) = intp_lodata(range(1))+cumsum(-intp_imuratedata(range))*.001;
               robot_state(range)= -1+robot_state(range); %% outer leg swings
            calc_swtsptheitdata(range) = foot_radius_inner -d_ac_inner*cos(intp_fidata(range)-curr_slope)+ L_leg*(cos(intp_lidata(range)-curr_slope) -cos(intp_lodata(range)-curr_slope))- d_ankle2sweetspot*cos(intp_fodata(range)-curr_slope);
            calc_sfhdata(range) = foot_radius_inner -d_ac_inner*cos(intp_fidata(range)-curr_slope)+ L_leg*(cos(intp_lidata(range)-curr_slope) -cos(intp_lodata(range)-curr_slope)) + d_ac_outer*cos(intp_fodata(range)-curr_slope) - foot_radius_outer ; 
            calc_swtsptheit1resetdata(range) = foot_radius_inner -d_ac_inner*cos(intp_fi1resetdata(range)-curr_slope)+ L_leg*(cos(intp_li1resetdata(range)-curr_slope) -cos(intp_lo1resetdata(range)-curr_slope))- d_ankle2sweetspot*cos(intp_fo1resetdata(range)-curr_slope);
            calc_sfh1resetdata(range) = foot_radius_inner -d_ac_inner*cos(intp_fi1resetdata(range)-curr_slope)+ L_leg*(cos(intp_li1resetdata(range)-curr_slope) -cos(intp_lo1resetdata(range)-curr_slope)) + d_ac_outer*cos(intp_fo1resetdata(range)-curr_slope) - foot_radius_outer ; 
            
        end   
        
end

%% floor height
% flrheit_ol_data = floor_height([0; distancedata], .263*ones(size([0; distancedata]))); % floorheit Oleft foot (near ramp)(robo walking on middle of track from x=0 to 5m)
% flrheit_il_data = floor_height([0; distancedata],  .09*ones(size([0; distancedata]))); % floorheit Ileft foot (near ramp)
% flrheit_ir_data = floor_height([0; distancedata], -.09*ones(size([0; distancedata]))); % floor height inner right foot (near table)
% flrheit_or_data = floor_height([0; distancedata],-.263*ones(size([0; distancedata]))); % floor height outer right foot (near table)
% flrheit_increments = zeros(size(distancedata));% initialize
% for i = 2:length(distancetime)+1
%         if(rem(i,2)==0)
%             flrheit_increments(i-1) = (flrheit_ir_data(i) + flrheit_il_data(i) - flrheit_ol_data(i-1) - flrheit_or_data(i-1))/2;  %% floor height increment
%         else
%             flrheit_increments(i-1) = (flrheit_ol_data(i) + flrheit_or_data(i) - flrheit_ir_data(i-1) - flrheit_il_data(i-1))/2;
%         end      
% end


%% estimating actual times of heel strike
% meandata= zeros(size(time_interval)); % running average (exp filter)
% meanplusdev = zeros(size(time_interval)); % running standard deviation (exp filter)
% 
% innerswing = 1123.3435; outerswing = 2332.34534; first_time_flag = 1; first_rise_flag=1;  % random numners emulating an enum
% 
% state = innerswing; % initialize
% counter = 0;  hs_index_data = []; hs_detect_data = [];
% filtercoeff= .9;
% for i=1:length(time_interval);
%     if(intp_hipdata(i)>0 && state==innerswing )
%             if(first_time_flag==1)
%              prev_innerfoot_mean = intp_fisensedata(i); innerfoot_mean = intp_fisensedata(i); 
%              prev_innerfoot_dev = 0;       innerfoot_dev = 0;        
%              first_time_flag=0;
%             end
%             if (intp_fisensedata(i)>innerfoot_mean+innerfoot_dev  &&  first_rise_flag==1)
%                 hs_index = i; first_rise_flag=0; storedmeanplusdev = innerfoot_mean+innerfoot_dev ; storedmean = innerfoot_mean;
%             elseif (first_rise_flag==0)
%                 if(intp_fisensedata(i)>storedmean+2000)
%                     counter = counter+1;
%                     if (counter>2)
%                         state=outerswing;
%                         first_time_flag=1; first_rise_flag=1;
%                         counter = 0;
%                         hs_index_data = [hs_index_data, hs_index];
%                     end  
%                 elseif(intp_fisensedata(i)>intp_fisensedata(i-1))
%                     counter =0;
%                 elseif(intp_fisensedata(i)<intp_fisensedata(i-1))
%                     counter =0;
%                     first_rise_flag=1; 
%                 end
%             end
%             innerfoot_mean = filtercoeff*prev_innerfoot_mean + (1-filtercoeff)*intp_fisensedata(i); 
%             innerfoot_dev  = filtercoeff*prev_innerfoot_dev  + 5*(1-filtercoeff)*abs(intp_fisensedata(i)-innerfoot_mean);
%             meandata(i) = innerfoot_mean;
%             meanplusdev(i) = innerfoot_mean+innerfoot_dev;
%             prev_innerfoot_mean = innerfoot_mean;
%             prev_innerfoot_dev = innerfoot_dev;
%     elseif (intp_hipdata(i)<0 && state==outerswing)
%              if(first_time_flag==1)
%              prev_outerfoot_mean = intp_fosensedata(i); outerfoot_mean = intp_fosensedata(i); 
%              prev_outerfoot_dev = 0;        outerfoot_dev = 0;     
%              first_time_flag=0;
%             end
%             if (intp_fosensedata(i)>outerfoot_mean+outerfoot_dev  &&  first_rise_flag==1)
%                 hs_index = i; first_rise_flag=0; storedmeanplusdev = outerfoot_mean+outerfoot_dev; storedmean = outerfoot_mean;
%             elseif (first_rise_flag==0)
%                 if(intp_fosensedata(i)>storedmean+2000)
%                     counter = counter+1;
%                     if (counter>2)
%                         state=innerswing;
%                         first_time_flag=1; first_rise_flag=1;
%                         counter = 0;
%                         hs_index_data = [hs_index_data, hs_index];
%                     end  
%                 elseif(intp_fosensedata(i)>intp_fosensedata(i-1))
%                     counter =0;
%                 elseif(intp_fosensedata(i)<intp_fosensedata(i-1))
%                     counter =0;
%                     first_rise_flag=1; 
%                 end
%             end
%             outerfoot_mean = filtercoeff*prev_outerfoot_mean + (1-filtercoeff)*intp_fosensedata(i); 
%             outerfoot_dev  = filtercoeff*prev_outerfoot_dev  + 5*(1-filtercoeff)*abs(intp_fosensedata(i)-outerfoot_mean);
%             meandata(i) = outerfoot_mean;
%             meanplusdev(i) = outerfoot_mean+outerfoot_dev;
%             prev_outerfoot_mean = outerfoot_mean;
%             prev_outerfoot_dev  = outerfoot_dev;
%         
%     end
% end
% hs_accurate_times = time_interval(hs_index_data);
% % 
time_to_hs = 120;
 fig2 = figure(2); hold on; title('inner foot hits');xlabel('time to actual hs in ms'); ylabel('swt spot height in m');
 axis ([-40 120 -.011 .05]); set(fig2, 'position', [100 100 500 500]);
 fig3 = figure(3); hold on; title('outer foot hits');xlabel('time to actual hs in ms'); ylabel('swt spot height in m');
axis ([-40 120 -.011 .05]); set(fig3, 'position', [650 100 500 500]);
% % % 
% % figure(4); hold on; title('inner foot hits');xlabel('time to actual hs in ms'); ylabel('predicted time to hs in ms');
% % figure(5); hold on; title('outer foot hits');xlabel('time to actual hs in ms'); ylabel('predicted time to hs in ms');
% 
% 
% 
inner_prepush_times= fi_fsmtime(fi_fsmdata ==1); 
outer_prepush_times= fo_fsmtime(fo_fsmdata ==1);
% 
% 
% 
 deltimei = []; deltimeo = [];
% 
% if(length(hs_accurate_times) == length(distancetime)-start_index+1)
%     display('hs_accurate times is making sense, they are same in number to hs detections')
% else
%     display('hs_accurate times are not commensurate to distance times')
% end

hs_accurate_times = distancetime(start_index:end)-12; 

for i= 1:length(hs_accurate_times);
   pause(.1)
   % index = find((time_interval >= hs_accurate_times(i)-time_to_hs)& (time_interval <= hs_accurate_times(i)));
   % data = calc_swtsptheitdata(index);
   % time = time_to_hs:-1:0;
   
    index = find((time_interval >= hs_accurate_times(i)-time_to_hs)& (time_interval < distancetime(i+start_index-1)));
    deltimei = [deltimei hs_accurate_times(i)-inner_prepush_times((inner_prepush_times >= hs_accurate_times(i)-time_to_hs-100)& (inner_prepush_times < distancetime(i+start_index-1)+10))];
    deltimeo = [deltimeo hs_accurate_times(i)-outer_prepush_times((outer_prepush_times >= hs_accurate_times(i)-time_to_hs-100)& (outer_prepush_times < distancetime(i+start_index-1)+10))];
    
    data = calc_swtsptheitdata(index);
    time = time_to_hs:-1:-length(data)+time_to_hs+1;
    
    if(~isempty(index))
        if (intp_hipdata(index(1))> 0 )
          figure(2)
          plot(time , data ,  col, time(end), data(end),'g.')
          axis ([-40 120 -.011 .05])
    

        elseif (intp_hipdata(index(1))< 0 )
          figure(3) 
          plot(time , data , col, time(end), data(end),'g.')
          axis ([-40 120 -.011 .05])
          
        end
    end
end

%deltimei(1)=[]; % remove the first data point its first step and id bad
deltimei   % how much in future by ms the prepush occurs when inner foot hits
display(['mean prepush time when fi hits ' num2str(mean(deltimei)) 'ms, dev ' num2str(std(deltimei)) 'ms.'])
deltimeo   % same for outer
display(['mean prepush time when fo hits ' num2str(mean(deltimeo)) 'ms, dev ' num2str(std(deltimeo)) 'ms.'])


figure(4)
hist(deltimei,20)
title('inner foot hits')
figure(5)
hist(deltimeo,20)
title('outer foot hits')

% hs_detect_delay = distancetime(start_index:end)'-hs_accurate_times
% display(['mean hs detection delay ' num2str(mean(hs_detect_delay))  ', dev ' num2str(std(hs_detect_delay))])

numnum=2;
power_int_timerange = distancetime(numnum):distancetime(end);
intp_hip_powerdata = interp1(hip_powertime,hip_powerdata,power_int_timerange);
intp_fi_powerdata = interp1(fi_powertime,fi_powerdata,power_int_timerange);
intp_fo_powerdata = interp1(fo_powertime,fo_powerdata,power_int_timerange);

total_time = length(power_int_timerange);
total_work = (sum(intp_hip_powerdata)+sum(intp_fi_powerdata)+sum(intp_fo_powerdata))*.001;
mean = mean(intp_hip_powerdata)+mean(intp_fi_powerdata) + mean(intp_fo_powerdata);
total_distance = (distancedata(end)-distancedata(numnum));
COT = (total_work + overheads*total_time*.001)/(robot_mass*9.8*total_distance); 
display(['COT is ' num2str(COT)])
display(['avg vel is ' num2str((distancedata(end-2)-distancedata(start_index+1))/(distancetime(end-2)-distancetime(start_index+1))*1000)])

%% plots
fig = figure(1); n=2; obj = datacursormode(fig); set(obj,'Updatefcn',@cursorprecision);
hold on;
% plot(litime, lidata, '-b.', 'displayname', 'li');
plot(fitime, fidata-foot_absang_calibration_constant, 'm.', 'displayname', 'fi');
plot(fotime, fodata-foot_absang_calibration_constant, 'r.', 'displayname', 'fo');
% plot(time_interval, intp_lodata-intp_hipdata, '-b', 'displayname', 'calc li' );
% plot(time_interval, intp_lodata+(intp_orankdata)-ankle_angle_offset, '-r', 'displayname', 'calc fo' );
% plot(time_interval, intp_lodata-intp_hipdata+intp_iankdata-ankle_angle_offset, '-m', 'displayname', 'calc fi' );
 
% plot(fisensetime, fisensedata/10000, '-c.', 'displayname', 'Inner foot sensor');
% plot(fosensetime, fosensedata/10000, '-b.', 'displayname', 'Outer foot sensor');
% plot(time_interval, meandata/10000, 'g', time_interval, meanplusdev/10000, 'r')



plot(hs_accurate_times, zeros(size(hs_accurate_times)), 'go', 'displayname', 'Actual HS times')
plot(inner_prepush_times, 0*inner_prepush_times, 'co')
plot(outer_prepush_times, 0*outer_prepush_times, 'bo')

%  plot(hiptime, hipdata, '-b.', 'displayname', 'hip');
%  plot(ianktime, iankdata, 'm.', 'displayname', 'iank');
%  plot(oranktime, orankdata, 'r.', 'displayname', 'orank');
% plot(olanktime, olankdata, 'r.', 'displayname', 'olank');
% plot(lotime, lodata, 'g.', 'displayname', 'lo');
% plot(lo1resettime, lo1resetdata, 'k.', 'displayname', 'lo resetted only once');
% plot(time_interval, calc_lodata, 'm.') %*******% BY INTEGRATING IMU RATE

% plot(time_interval, intp_hipdata, '-b', 'displayname', 'intp hip');
% plot(time_interval, intp_iankdata, '-m', 'displayname', 'intp iank');
% plot(time_interval, intp_orankdata, '-r', 'displayname', 'intp orank');
% plot(time_interval, intp_olankdata, ':r', 'displayname', 'inpt olank');
% plot(time_interval, intp_lodata, '-m', 'displayname', 'intp lo');
% plot(time_interval, intp_lo1resetdata, '-k', 'displayname', 'intp lo resetted only once');
% plot(time_interval, intp_lidata, '-r', 'displayname', 'intp li');
plot(time_interval, intp_fodata, '-r', 'displayname', 'intp fo');
plot(time_interval, intp_fidata, '-m', 'displayname', 'intp fi');

% plot(hipratetime, hipratedata, '-k.', 'displayname', 'hiprate');
% plot(imuratetime, imuratedata, '-k.', 'displayname', 'imurate');
% plot(hratetime, hratedata, '-g.', 'displayname', 'hrate');
% plot(ehratetime, ehratedata, '-m.', 'displayname', 'ehrate');
% plot(time_to_hstime, time_to_hsdata, '-g.', 'displayname','time to hs');
% plot(sfhtime, sfhdata*100, '-k.', 'displayname', 'sfh');
% plot(swtsptheittime, swtsptheitdata*100, 'k.', 'displayname', 'swtsptheit');

% plot(fi_fsmtime, fi_fsmdata/10, '-c.', 'displayname', 'fi_fsm');
% plot(fo_fsmtime, fo_fsmdata/10, '-b.', 'displayname', 'fo_fsm');

% plot(time_interval, calc_sfhdata*100, '-r', 'displayname', 'calc sfh');
% plot(time_interval, calc_swtsptheitdata*100, '-m', 'displayname', 'calc swtsptheit');
% plot(time_interval, calc_sfh1resetdata*100, '-m', 'displayname', 'once reset sfh');
% plot(time_interval, calc_swtsptheit1resetdata*100, '-.r.', 'displayname', 'once reset swtsptheit');

plot(distancetime, zeros(size(distancetime)), 'ro', 'displayname', 'Measured hs times');
plot(time_interval, robot_state/10, '-r', 'displayname', 'state:1=inner swg, -1= outer swg');
% plot(distancetime, flrheit_increments*100, '-b*', 'displayname', 'floor height increments');
% plot(int_lo_resettime, [int_lo_resetdata], '-k.', 'displayname', 'int_lo_reset');
% plot(distancetime, distancedata, '-k.', 'displayname', 'distance');
% plot(slopetime, slopedata, '-go', 'displayname', 'slope');
% plot(distancetime, robotveldata, ':k.', 'displayname', 'robot velocity');
% plot(distancetime, .5*(flrheit_ol_data+flrheit_or_data), '-k.', 'displayname', 'FO floor height ');
% plot(distancetime, .5*(flrheit_il_data+flrheit_ir_data), '-b.', 'displayname', 'FI floor height ');
% 
% plot(hip_powertime, hip_powerdata/20, 'r.', 'displayname', 'hip power')
% plot(fi_powertime, fi_powerdata/20, 'c.', 'displayname', 'fi power')
% plot(fo_powertime, fo_powerdata/20, 'b.', 'displayname', 'fo power')
% plot(power_int_timerange, intp_hip_powerdata/20, '-r', 'displayname', 'intp hip power')
% plot(power_int_timerange, intp_fi_powerdata/20, '-c', 'displayname', 'intp fi power')
% plot(power_int_timerange, intp_fo_powerdata/20, '-b', 'displayname', 'intp fo power')

% plot(ficurrenttime, ficurrentdata, '-c.', 'displayname', 'fi current')
% plot(focurrenttime, focurrentdata, '-b.', 'displayname', 'fo current')

legend show
grid on
%linkaxes(ax,'x') % for subplots



