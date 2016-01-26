function Parameters = Extract_Parameters(robot)

%
%FUNCTION:
% This function unpacks Pranav's 'robot' struct into a single vector of
% parameters that are used by all of the C++ routines.
%

%In Pranav's code, he has a vector called parms. This vector is comprised
%of [GL_DIM, GL_MOT, ...] where the ... is a motor coefficients vector. I
%assembly the motor coefficients vector seperately and concatenate it right
%before the functions are called in either Single_Stance_Step or
%Double_Stance_Step.

%% GL_DIM  (Robot dimensions - Pranav's code)

M = robot.parm.M; 
m = robot.parm.m; 
c = robot.parm.c;
I = robot.parm.I;  
g = robot.parm.g; 
l = robot.parm.l; 
r = robot.parm.r;  
d = robot.parm.d; 
gam = robot.parm.gam;
w = robot.parm.w; 
k = robot.parm.k; 
J = robot.parm.J;
k_st = robot.parm.k_st;
k_sw = robot.parm.k_sw;
C_st = robot.parm.C_st;
C_sw = robot.parm.C_sw;

%% GL_MOT (Robot motor parameters) = [R V0 I0 G_h G_a K I_MAX e_r mu_h C1_h C0_h mu_st C1_st C0_st C1m_st C0m_st ElectricalW eps];

R = robot.motor.R; 
V0 = robot.motor.V0;
I0 = robot.motor.I0;
G_h = robot.motor.G_h; 
G_a = robot.motor.G_a;
K = robot.motor.K; 
I_MAX = robot.motor.I_MAX; 
e_r = robot.motor.e_r;
mu_h = robot.motor.mu_h; 
C1_h = robot.motor.C1_h; 
C0_h = robot.motor.C0_h; 
mu_st = robot.motor.mu_st; 
C1_st = robot.motor.C1_st; 
C0_st = robot.motor.C0_st;
C1m_st = robot.motor.C1m_st;
C0m_st = robot.motor.C0m_st;
ElectricalW = robot.motor.ElectricalW;
eps = robot.motor.eps;
eps_f = robot.motor.eps_f;

%% Assemble into a stupidly long vector

Parameters = [M,m,I,l,c,w,r,d,g,k,gam,J,k_st,k_sw,C_st,C_sw ...
          R,V0,I0,G_h,G_a,K,I_MAX,e_r,mu_h,C1_h,C0_h,mu_st,C1_st,C0_st,C1m_st,C0m_st,ElectricalW,eps,eps_f];
      
end