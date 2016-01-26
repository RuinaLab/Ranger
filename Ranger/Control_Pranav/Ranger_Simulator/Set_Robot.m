function [Motor, Parm] = Set_Robot()

%This is a function file written by Pranav that returns the robot struct
%that is used in the simulator. 

%Slight modifications made by MPK

%Motor is a struct of motor parameters
%Parm is a struct of the physical parameters of the robot
%robot.sim has been moved to the primary parameters file

friction_flag = 1; %1 to include joint/motor friction

%%%%%%%% Motor properties %%%%%%%%%%%%%%
%GL_MOT = [R V0 I0 G_h G_a K I_MAX e_r mu_h C1_h C0_h mu_st C1_st C0_st C1m_st C0m_st ElectricalW eps];
Motor.R = 1; %1.3; 
Motor.V0 = 1.5; 
Motor.I0 = 0.01;
Motor.G_h = 66; 
Motor.G_a = 34; %43;
Motor.K = 0.018; %0.024;%0.017; 
Motor.I_MAX = 8; 
Motor.e_r = 0; %regeneration effeciency. 0 for no regeneration and 1 for full regeneration

if (friction_flag == 0)
    %%%%% without friction %%%%%
    Motor.mu_h = 0; %0.15
    Motor.C1_h = 0; %0.12
    Motor.C0_h = 0; %0.12 
    Motor.mu_st = 0; %0.1
    Motor.C1_st = 0; %0.0
    Motor.C0_st = 0; %0.0
    Motor.C1m_st = 0; %0.03
    Motor.C0m_st = 0; %0.1
elseif  (friction_flag == 1)
    %%%%% with friction %%%%%
    Motor.mu_h = 0.1;
    Motor.C1_h = 0.1;
    Motor.C0_h = 0.1;
    Motor.mu_st = 0.1;
    Motor.C1_st = 0.0;
    Motor.C0_st = 0.0;
    Motor.C1m_st = 0.01;
    Motor.C0m_st = 0.1;
else
    error('check friction_flag');
end

%Motor.ElectricalW = 4.25 + 0.5 + 0.6; %electrical overheads + steering + h-bridge
%Motor.ElectricalW = 4.7 + 0.5 + 0.6; %electrical overheads + steering + h-bridge
Motor.ElectricalW = 4.7 + 0.0 + 0.45; %electrical overheads + steering + h-bridge %removed steering W 6/20/2011
Motor.eps = 0.1;
Motor.eps_f = 100;
Motor.Eankle.b0 = 0; %0.850013847814768;
Motor.Eankle.b1 = 1.06 ; %-0.214598273470477;
Motor.Eankle.b2 =  0; %0.695591530093978;

%%%%%%%%% Mass properties %%%%%%%%%%%%%%%%%
%GL_DIM = [M m I l c w r d g k gam J k_st k_sw C_st C_sw];
m_tot = 9.91; %8.46;
mc_in = 0.72;
I_in = 0.53;
%i.e. M+2*m = 8.46, mr_in = 0.72, I_hinge_in = 0.53

per = 0.50000; % 0 < per <= 0.5 //fraction mass in leg, 0.5 means weight is equally distributed among legs

Parm.M = (1-2*per)*m_tot; %4.5;
Parm.m = per*m_tot;%2;
Parm.c = mc_in/Parm.m; %0.3;
Parm.I = I_in-(mc_in*mc_in/Parm.m); %0.238;  
Parm.g = 9.81; 
Parm.l = 0.96;
Parm.r = 0.2;  
Parm.d = 0.11;%0.11; 
Parm.gam = 0.00;%0;
Parm.w = 0;%0.03;
Parm.k = 7.6;%6 
Parm.J = 34*34*1.6*1e-6; %G_a^2*(1.6^2)*1e-6;
Parm.k_st = 14;
Parm.k_sw = 14;
Parm.C_sw = 0.1;
Parm.C_st = 0.1;

%%%%%%%%%%
%robot.lift.q4 = -0.2;
% robot.lock.q2 = -0.1;
% robot.lock.q4 = -0.1;

end